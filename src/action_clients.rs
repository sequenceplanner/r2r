use futures::channel::{mpsc, oneshot};
use futures::future::{FutureExt, TryFutureExt};
use futures::stream::Stream;
use std::collections::HashMap;
use std::ffi::CString;
use std::future::Future;
use std::mem::MaybeUninit;
use std::sync::{Mutex, Weak};

use crate::action_common::*;
use crate::error::*;
use crate::msg_types::generated_msgs::{action_msgs, builtin_interfaces, unique_identifier_msgs};
use crate::msg_types::*;
use r2r_actions::*;
use r2r_rcl::*;

unsafe impl<T> Send for ActionClient<T> where T: WrappedActionTypeSupport {}

/// Action client
///
/// Use this to make goal requests to an action server.
#[derive(Clone)]
pub struct ActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    client: Weak<Mutex<WrappedActionClient<T>>>,
}

unsafe impl<T> Send for ActionClientGoal<T> where T: WrappedActionTypeSupport {}

/// Action client goal handle
///
/// This can be used to cancel goals and query the status of goals.
#[derive(Clone)]
pub struct ActionClientGoal<T>
where
    T: WrappedActionTypeSupport,
{
    client: Weak<Mutex<WrappedActionClient<T>>>,
    pub uuid: uuid::Uuid,
}

impl<T: 'static> ActionClientGoal<T>
where
    T: WrappedActionTypeSupport,
{
    /// Get the current status of this goal.
    pub fn get_status(&self) -> Result<GoalStatus> {
        let client = self
            .client
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_CLIENT_INVALID)?;
        let client = client.lock().unwrap();

        Ok(client.get_goal_status(&self.uuid))
    }

    /// Send a cancel request for this goal to the server.
    ///
    /// If the server accepts and completes the request, the future completes without error.
    /// Otherwise, one of these errors can be returned:
    /// - `GoalCancelRejected`
    /// - `GoalCancelUnknownGoalID`
    /// - `GoalCancelAlreadyTerminated`
    pub fn cancel(&self) -> Result<impl Future<Output = Result<()>>> {
        // upgrade to actual ref. if still alive
        let client = self
            .client
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();

        client.send_cancel_request(&self.uuid)
    }
}

impl<T: 'static> ActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    /// Make a new goal request.
    ///
    /// If the server accepts the new goal, the future resolves to a triple of:
    /// - A goal handle.
    /// - A new future for the eventual result.
    /// - A stream of feedback messages.
    pub fn send_goal_request(
        &self,
        goal: T::Goal,
    ) -> Result<
        impl Future<
            Output = Result<(
                ActionClientGoal<T>,
                impl Future<Output = Result<(GoalStatus, T::Result)>>,
                impl Stream<Item = T::Feedback> + Unpin,
            )>,
        >,
    >
    where
        T: WrappedActionTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let client = self
            .client
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();

        let uuid = uuid::Uuid::new_v4();
        let uuid_msg = unique_identifier_msgs::msg::UUID {
            uuid: uuid.as_bytes().to_vec(),
        };
        let request_msg = T::make_goal_request_msg(uuid_msg, goal);
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Request,
        >::from(&request_msg);
        let mut seq_no = 0i64;
        let result = unsafe {
            rcl_action_send_goal_request(&client.rcl_handle, native_msg.void_ptr(), &mut seq_no)
        };

        // set up channels
        let (goal_req_sender, goal_req_receiver) =
            oneshot::channel::<(bool, builtin_interfaces::msg::Time)>();
        let (feedback_sender, feedback_receiver) = mpsc::channel::<T::Feedback>(10);
        client.feedback_senders.push((uuid, feedback_sender));
        let (result_sender, result_receiver) = oneshot::channel::<(GoalStatus, T::Result)>();
        client.result_senders.push((uuid, result_sender));

        if result == RCL_RET_OK as i32 {
            client
                .goal_response_channels
                .push((seq_no, goal_req_sender));
            // instead of "canceled" we return invalid client.
            let fut_client = Weak::clone(&self.client);
            let future = goal_req_receiver
                .map_err(|_| Error::RCL_RET_ACTION_CLIENT_INVALID)
                .map(move |r| match r {
                    Ok((accepted, _stamp)) => {
                        if accepted {
                            // on goal accept we immediately send the result request
                            {
                                let c = fut_client
                                    .upgrade()
                                    .ok_or(Error::RCL_RET_ACTION_CLIENT_INVALID)?;
                                let mut c = c.lock().unwrap();
                                c.send_result_request(uuid);
                            }

                            Ok((
                                ActionClientGoal {
                                    client: fut_client,
                                    uuid,
                                },
                                result_receiver.map_err(|_| Error::RCL_RET_ACTION_CLIENT_INVALID),
                                feedback_receiver,
                            ))
                        } else {
                            Err(Error::RCL_RET_ACTION_GOAL_REJECTED)
                        }
                    }
                    Err(e) => Err(e),
                });
            Ok(future)
        } else {
            eprintln!("coult not send goal request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}

pub fn make_action_client<T>(client: Weak<Mutex<WrappedActionClient<T>>>) -> ActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    ActionClient { client }
}

pub type ResultSender<R> = (uuid::Uuid, oneshot::Sender<(GoalStatus, R)>);
pub struct WrappedActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    pub rcl_handle: rcl_action_client_t,
    pub goal_response_channels: Vec<(i64, oneshot::Sender<(bool, builtin_interfaces::msg::Time)>)>,
    pub cancel_response_channels:
        Vec<(i64, oneshot::Sender<action_msgs::srv::CancelGoal::Response>)>,
    pub feedback_senders: Vec<(uuid::Uuid, mpsc::Sender<T::Feedback>)>,
    pub result_requests: Vec<(i64, uuid::Uuid)>,
    pub result_senders: Vec<ResultSender<T::Result>>,
    pub goal_status: HashMap<uuid::Uuid, GoalStatus>,

    pub poll_available_channels: Vec<oneshot::Sender<()>>,
}

pub trait ActionClient_ {
    fn handle(&self) -> &rcl_action_client_t;
    fn destroy(&mut self, node: &mut rcl_node_t);

    fn handle_goal_response(&mut self);
    fn handle_cancel_response(&mut self);
    fn handle_feedback_msg(&mut self);
    fn handle_status_msg(&mut self);
    fn handle_result_response(&mut self);

    fn send_result_request(&mut self, uuid: uuid::Uuid);

    fn register_poll_available(&mut self, s: oneshot::Sender<()>);
    fn poll_available(&mut self, node: &mut rcl_node_t);
}

impl<T> WrappedActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    pub fn get_goal_status(&self, uuid: &uuid::Uuid) -> GoalStatus {
        *self.goal_status.get(uuid).unwrap_or(&GoalStatus::Unknown)
    }

    pub fn send_cancel_request(
        &mut self,
        goal: &uuid::Uuid,
    ) -> Result<impl Future<Output = Result<()>>>
    where
        T: WrappedActionTypeSupport,
    {
        let msg = action_msgs::srv::CancelGoal::Request {
            goal_info: action_msgs::msg::GoalInfo {
                goal_id: unique_identifier_msgs::msg::UUID {
                    uuid: goal.as_bytes().to_vec(),
                },
                ..action_msgs::msg::GoalInfo::default()
            },
        };
        let native_msg = WrappedNativeMsg::<action_msgs::srv::CancelGoal::Request>::from(&msg);
        let mut seq_no = 0i64;
        let result = unsafe {
            rcl_action_send_cancel_request(&self.rcl_handle, native_msg.void_ptr(), &mut seq_no)
        };

        if result == RCL_RET_OK as i32 {
            let (cancel_req_sender, cancel_req_receiver) =
                oneshot::channel::<action_msgs::srv::CancelGoal::Response>();

            self.cancel_response_channels
                .push((seq_no, cancel_req_sender));
            // instead of "canceled" we return invalid client.
            let future = cancel_req_receiver
                .map_err(|_| Error::RCL_RET_CLIENT_INVALID)
                .map(|r| match r {
                    Ok(r) => match r.return_code {
                        0 => Ok(()),
                        1 => Err(Error::GoalCancelRejected),
                        2 => Err(Error::GoalCancelUnknownGoalID),
                        3 => Err(Error::GoalCancelAlreadyTerminated),
                        x => panic!("unknown error code return from action server: {}", x),
                    },
                    Err(e) => Err(e),
                });
            Ok(future)
        } else {
            eprintln!("coult not send goal request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}

impl<T: 'static> ActionClient_ for WrappedActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    fn handle(&self) -> &rcl_action_client_t {
        &self.rcl_handle
    }

    fn handle_goal_response(&mut self) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut response_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
        >::new();

        let ret = unsafe {
            rcl_action_take_goal_response(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                response_msg.void_ptr_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let request_id = unsafe { request_id.assume_init() };
            if let Some(idx) = self
                .goal_response_channels
                .iter()
                .position(|(id, _)| id == &request_id.sequence_number)
            {
                let (_, sender) = self.goal_response_channels.swap_remove(idx);
                let response = <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response::from_native(&response_msg);
                let (accept, stamp) = T::destructure_goal_response_msg(response);
                match sender.send((accept, stamp)) {
                    Ok(()) => {}
                    Err(e) => {
                        println!("error sending to action client: {:?}", e);
                    }
                }
            } else {
                let we_have: String = self
                    .goal_response_channels
                    .iter()
                    .map(|(id, _)| id.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                eprintln!(
                    "no such req id: {}, we have [{}], ignoring",
                    request_id.sequence_number, we_have
                );
            }
        }
    }

    fn handle_cancel_response(&mut self) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut response_msg = WrappedNativeMsg::<action_msgs::srv::CancelGoal::Response>::new();

        let ret = unsafe {
            rcl_action_take_cancel_response(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                response_msg.void_ptr_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let request_id = unsafe { request_id.assume_init() };
            if let Some(idx) = self
                .cancel_response_channels
                .iter()
                .position(|(id, _)| id == &request_id.sequence_number)
            {
                let (_, sender) = self.cancel_response_channels.swap_remove(idx);
                let response = action_msgs::srv::CancelGoal::Response::from_native(&response_msg);
                if let Err(e) = sender.send(response) {
                    eprintln!("warning: could not send cancel response msg ({:?})", e)
                }
            } else {
                let we_have: String = self
                    .goal_response_channels
                    .iter()
                    .map(|(id, _)| id.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                eprintln!(
                    "no such req id: {}, we have [{}], ignoring",
                    request_id.sequence_number, we_have
                );
            }
        }
    }

    fn handle_feedback_msg(&mut self) {
        let mut feedback_msg = WrappedNativeMsg::<T::FeedbackMessage>::new();
        let ret =
            unsafe { rcl_action_take_feedback(&self.rcl_handle, feedback_msg.void_ptr_mut()) };
        if ret == RCL_RET_OK as i32 {
            let msg = T::FeedbackMessage::from_native(&feedback_msg);
            let (uuid, feedback) = T::destructure_feedback_msg(msg);
            let msg_uuid = uuid_msg_to_uuid(&uuid);
            if let Some((_, sender)) = self
                .feedback_senders
                .iter_mut()
                .find(|(uuid, _)| uuid == &msg_uuid)
            {
                if let Err(e) = sender.try_send(feedback) {
                    eprintln!("warning: could not send feedback msg ({})", e)
                }
            }
        }
    }

    fn handle_status_msg(&mut self) {
        let mut status_array = WrappedNativeMsg::<action_msgs::msg::GoalStatusArray>::new();
        let ret = unsafe { rcl_action_take_status(&self.rcl_handle, status_array.void_ptr_mut()) };
        if ret == RCL_RET_OK as i32 {
            let arr = action_msgs::msg::GoalStatusArray::from_native(&status_array);
            for a in &arr.status_list {
                let uuid = uuid_msg_to_uuid(&a.goal_info.goal_id);
                if !self.result_senders.iter().any(|(suuid, _)| suuid == &uuid) {
                    continue;
                }
                let status = GoalStatus::from_rcl(a.status);
                *self.goal_status.entry(uuid).or_insert(GoalStatus::Unknown) = status;
            }
        }
    }

    fn handle_result_response(&mut self) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut response_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::new();

        let ret = unsafe {
            rcl_action_take_result_response(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                response_msg.void_ptr_mut(),
            )
        };

        if ret == RCL_RET_OK as i32 {
            let request_id = unsafe { request_id.assume_init() };
            if let Some(idx) = self
                .result_requests
                .iter()
                .position(|(id, _)| id == &request_id.sequence_number)
            {
                let (_, uuid) = self.result_requests.swap_remove(idx);
                if let Some(idx) = self
                    .result_senders
                    .iter()
                    .position(|(suuid, _)| suuid == &uuid)
                {
                    let (_, sender) = self.result_senders.swap_remove(idx);
                    let response = <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response::from_native(&response_msg);
                    let (status, result) = T::destructure_result_response_msg(response);
                    let status = GoalStatus::from_rcl(status);
                    match sender.send((status, result)) {
                        Ok(()) => {}
                        Err(e) => {
                            println!("error sending result to action client: {:?}", e);
                        }
                    }
                }
            } else {
                let we_have: String = self
                    .result_requests
                    .iter()
                    .map(|(id, _)| id.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                eprintln!(
                    "no such req id: {}, we have [{}], ignoring",
                    request_id.sequence_number, we_have
                );
            }
        }
    }

    fn send_result_request(&mut self, uuid: uuid::Uuid) {
        let uuid_msg = unique_identifier_msgs::msg::UUID {
            uuid: uuid.as_bytes().to_vec(),
        };
        let request_msg = T::make_result_request_msg(uuid_msg);
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Request,
        >::from(&request_msg);
        let mut seq_no = 0i64;
        let result = unsafe {
            rcl_action_send_result_request(&self.rcl_handle, native_msg.void_ptr(), &mut seq_no)
        };

        if result == RCL_RET_OK as i32 {
            self.result_requests.push((seq_no, uuid));
        } else {
            eprintln!("coult not send request {}", result);
        }
    }

    fn register_poll_available(&mut self, s: oneshot::Sender<()>) {
        self.poll_available_channels.push(s);
    }

    fn poll_available(&mut self, node: &mut rcl_node_t) {
        if self.poll_available_channels.is_empty() {
            return;
        }
        let available = action_server_available_helper(node, self.handle());
        match available {
            Ok(true) => {
                // send ok and close channels
                while let Some(sender) = self.poll_available_channels.pop() {
                    let _res = sender.send(()); // we ignore if receiver dropped.
                }
            }
            Ok(false) => {
                // not available...
            }
            Err(_) => {
                // error, close all channels
                self.poll_available_channels.clear();
            }
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_action_client_fini(&mut self.rcl_handle, node);
        }
    }
}

pub fn create_action_client_helper(
    node: &mut rcl_node_t,
    action_name: &str,
    action_ts: *const rosidl_action_type_support_t,
) -> Result<rcl_action_client_t> {
    let mut client_handle = unsafe { rcl_action_get_zero_initialized_client() };
    let action_name_c_string =
        CString::new(action_name).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let client_options = rcl_action_client_get_default_options();
        rcl_action_client_init(
            &mut client_handle,
            node,
            action_ts,
            action_name_c_string.as_ptr(),
            &client_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(client_handle)
    } else {
        Err(Error::from_rcl_error(result))
    }
}

pub fn action_client_get_num_waits(
    rcl_handle: &rcl_action_client_t,
    num_subs: &mut usize,
    num_gc: &mut usize,
    num_timers: &mut usize,
    num_clients: &mut usize,
    num_services: &mut usize,
) -> Result<()> {
    unsafe {
        let result = rcl_action_client_wait_set_get_num_entities(
            rcl_handle,
            num_subs,
            num_gc,
            num_timers,
            num_clients,
            num_services,
        );
        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            Err(Error::from_rcl_error(result))
        }
    }
}

use crate::nodes::IsAvailablePollable;

impl<T: 'static> IsAvailablePollable for ActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    fn register_poll_available(&self, sender: oneshot::Sender<()>) -> Result<()> {
        let client = self
            .client
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();
        client.register_poll_available(sender);
        Ok(())
    }
}

pub fn action_server_available_helper(
    node: &rcl_node_t,
    client: &rcl_action_client_t,
) -> Result<bool> {
    let mut avail = false;
    let result = unsafe { rcl_action_server_is_available(node, client, &mut avail) };

    if result == RCL_RET_OK as i32 {
        Ok(avail)
    } else {
        eprintln!("coult not check if action server is available {}", result);
        Err(Error::from_rcl_error(result))
    }
}
