use futures::channel::{mpsc, oneshot};
use futures::future::FutureExt;
use futures::future::{join_all, JoinAll};
use futures::stream::Stream;
use std::collections::HashMap;
use std::ffi::CString;
use std::mem::MaybeUninit;
use std::sync::{Arc, Mutex, Weak};

use crate::action_common::*;
use crate::error::*;
use crate::msg_types::generated_msgs::{action_msgs, builtin_interfaces, unique_identifier_msgs};
use crate::msg_types::*;
use r2r_actions::*;
use r2r_rcl::*;

pub trait ActionServer_ {
    fn handle(&self) -> &rcl_action_server_t;
    fn handle_mut(&mut self) -> &mut rcl_action_server_t;
    fn handle_goal_request(&mut self, server: Arc<Mutex<dyn ActionServer_>>);
    fn send_completed_cancel_requests(&mut self);
    fn handle_cancel_request(&mut self);
    fn handle_result_request(&mut self);
    fn handle_goal_expired(&mut self);
    fn publish_status(&self);
    fn set_goal_state(
        &mut self,
        uuid: &uuid::Uuid,
        new_state: rcl_action_goal_event_t,
    ) -> Result<()>;
    fn add_result(&mut self, uuid: uuid::Uuid, msg: Box<dyn VoidPtr>);
    fn cancel_goal(&mut self, uuid: &uuid::Uuid);
    fn is_cancelling(&self, uuid: &uuid::Uuid) -> Result<bool>;
    fn add_goal_handle(&mut self, uuid: uuid::Uuid, goal_handle: *mut rcl_action_goal_handle_t);
    fn destroy(&mut self, node: &mut rcl_node_t);
}

/// Request to cancel an active goal.
pub struct ActionServerCancelRequest {
    pub uuid: uuid::Uuid,
    response_sender: oneshot::Sender<(uuid::Uuid, bool)>,
}

impl ActionServerCancelRequest {
    /// Accepts the cancel request. The action server should now cancel the corresponding goal.
    pub fn accept(self) {
        if self.response_sender.send((self.uuid, true)).is_err() {
            eprintln!("warning: could not send goal canellation accept msg")
        }
    }
    /// Rejects the cancel request.
    pub fn reject(self) {
        if self.response_sender.send((self.uuid, false)).is_err() {
            eprintln!("warning: could not send goal cancellation rejection")
        }
    }
}

/// Request to the action server to accept a new `Goal`.
pub struct ActionServerGoalRequest<T>
where
    T: WrappedActionTypeSupport,
{
    pub uuid: uuid::Uuid,
    pub goal: T::Goal,
    cancel_requests: mpsc::Receiver<ActionServerCancelRequest>,
    server: Weak<Mutex<dyn ActionServer_>>,
    request_id: rmw_request_id_t,
}

unsafe impl<T> Send for ActionServerGoalRequest<T> where T: WrappedActionTypeSupport {}

impl<T: 'static> ActionServerGoalRequest<T>
where
    T: WrappedActionTypeSupport,
{
    /// Accept the goal request and become a ServerGoal.
    /// Returns a handle to the goal and a stream on which cancel requests can be received.
    pub fn accept(
        mut self,
    ) -> Result<(
        ActionServerGoal<T>,
        impl Stream<Item = ActionServerCancelRequest> + Unpin,
    )> {
        let uuid_msg = unique_identifier_msgs::msg::UUID {
            uuid: self.uuid.as_bytes().to_vec(),
        };
        let time = builtin_interfaces::msg::Time::default();
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: uuid_msg,
            stamp: time.clone(),
        };
        let native_goal_info = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        let server = self.server.upgrade().unwrap(); // todo fixme
        let mut server = server.lock().unwrap();

        let goal_handle: *mut rcl_action_goal_handle_t =
            unsafe { rcl_action_accept_new_goal(server.handle_mut(), &*native_goal_info) };

        // send response
        let response_msg = T::make_goal_response_msg(true, time);
        let mut response_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
        >::from(&response_msg);

        let ret = unsafe {
            rcl_action_send_goal_response(
                server.handle_mut(),
                &mut self.request_id,
                response_msg.void_ptr_mut(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            return Err(Error::from_rcl_error(ret));
        }

        unsafe {
            rcl_action_update_goal_state(goal_handle, rcl_action_goal_event_t::GOAL_EVENT_EXECUTE);
        }

        server.publish_status();

        let g = ActionServerGoal {
            uuid: self.uuid,
            goal: self.goal,
            server: self.server,
        };

        // server.goals.insert(g.uuid.clone(), goal_handle);
        server.add_goal_handle(g.uuid, goal_handle);

        Ok((g, self.cancel_requests))
    }

    /// reject the goal request and be consumed in the process
    pub fn reject(mut self) -> Result<()> {
        let time = builtin_interfaces::msg::Time::default();
        let server = self.server.upgrade().unwrap(); // todo fixme
        let mut server = server.lock().unwrap();

        let response_msg = T::make_goal_response_msg(false, time);
        let mut response_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
        >::from(&response_msg);

        let ret = unsafe {
            rcl_action_send_goal_response(
                server.handle_mut(),
                &mut self.request_id,
                response_msg.void_ptr_mut(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            return Err(Error::from_rcl_error(ret));
        }

        Ok(())
    }
}
pub type ActiveCancelRequest = (
    rmw_request_id_t,
    action_msgs::srv::CancelGoal::Response,
    JoinAll<oneshot::Receiver<(uuid::Uuid, bool)>>,
);

pub struct WrappedActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    pub rcl_handle: rcl_action_server_t,
    pub clock_handle: Box<rcl_clock_t>,
    pub goal_request_sender: mpsc::Sender<ActionServerGoalRequest<T>>,
    pub cancel_senders: HashMap<uuid::Uuid, mpsc::Sender<ActionServerCancelRequest>>,
    pub active_cancel_requests: Vec<ActiveCancelRequest>,
    pub goals: HashMap<uuid::Uuid, *mut rcl_action_goal_handle_t>,
    pub result_msgs: HashMap<uuid::Uuid, Box<dyn VoidPtr>>,
    pub result_requests: HashMap<uuid::Uuid, Vec<rmw_request_id_t>>,
}

impl<T: 'static> ActionServer_ for WrappedActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    fn handle(&self) -> &rcl_action_server_t {
        &self.rcl_handle
    }

    fn handle_mut(&mut self) -> &mut rcl_action_server_t {
        &mut self.rcl_handle
    }

    fn is_cancelling(&self, uuid: &uuid::Uuid) -> Result<bool> {
        if let Some(handle) = self.goals.get(uuid) {
            let mut state = 0u8; // TODO: int8 STATUS_UNKNOWN   = 0;
            let ret = unsafe { rcl_action_goal_handle_get_status(*handle, &mut state) };

            if ret != RCL_RET_OK as i32 {
                println!("action server: Failed to get goal handle state: {}", ret);
                return Err(Error::from_rcl_error(ret));
            }
            return Ok(state == 3u8); // TODO: int8 STATUS_CANCELING
        }
        Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID)
    }

    fn cancel_goal(&mut self, uuid: &uuid::Uuid) {
        if let Some(handle) = self.goals.remove(uuid) {
            let ret = unsafe {
                rcl_action_update_goal_state(
                    handle,
                    rcl_action_goal_event_t::GOAL_EVENT_CANCEL_GOAL,
                )
            };

            if ret != RCL_RET_OK as i32 {
                println!(
                    "action server: could not cancel goal: {}",
                    Error::from_rcl_error(ret)
                );
            }
        }
    }

    fn set_goal_state(
        &mut self,
        uuid: &uuid::Uuid,
        new_state: rcl_action_goal_event_t,
    ) -> Result<()> {
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: unique_identifier_msgs::msg::UUID {
                uuid: uuid.as_bytes().to_vec(),
            },
            ..action_msgs::msg::GoalInfo::default()
        };
        let goal_info_native = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        // does this goal exist?
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(self.handle(), &*goal_info_native) };

        if !goal_exists {
            println!("tried to publish result without a goal");
            return Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID);
        }

        if let Some(handle) = self.goals.get(uuid) {
            // todo: error handling
            unsafe {
                rcl_action_update_goal_state(*handle, new_state);
            }

            // todo: error handling
            unsafe {
                rcl_action_notify_goal_done(self.handle());
            }

            // send out updated statues
            self.publish_status();

            Ok(())
        } else {
            Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID)
        }
    }

    fn send_completed_cancel_requests(&mut self) {
        let mut canceled = vec![];
        let mut responses = vec![];
        self.active_cancel_requests
            .retain_mut(|(request_id, msg, fut)| {
                let boxed = fut.boxed();
                if let Some(results) = boxed.now_or_never() {
                    let mut response_msg = msg.clone();
                    let requested_cancels = response_msg.goals_canceling.len();
                    for r in results {
                        match r {
                            Ok((uuid, do_cancel)) => {
                                // cancel goal and filter response msg.
                                if do_cancel {
                                    canceled.push(uuid);
                                }

                                response_msg.goals_canceling.retain(|goal_info| {
                                    let msg_uuid = uuid_msg_to_uuid(&goal_info.goal_id);
                                    do_cancel && msg_uuid == uuid
                                });
                            }
                            Err(oneshot::Canceled) => {
                                eprintln!("Warning, cancel request not handled!");
                                return false; // skip this request.
                            }
                        }
                    }

                    // check if all cancels were rejected.
                    if requested_cancels >= 1 && response_msg.goals_canceling.is_empty() {
                        response_msg.return_code = 1; // TODO: auto generate these (int8 ERROR_REJECTED=1)
                    }

                    responses.push((*request_id, response_msg));

                    false
                } else {
                    true
                }
            });

        canceled.iter().for_each(|uuid| self.cancel_goal(uuid));
        if !canceled.is_empty() {
            // at least one goal state changed, publish a new status message
            self.publish_status();
        }

        // send out responses
        for (mut request_id, response_msg) in responses {
            // send out response msg.
            let mut native_msg =
                WrappedNativeMsg::<action_msgs::srv::CancelGoal::Response>::from(&response_msg);
            let ret = unsafe {
                rcl_action_send_cancel_response(
                    &self.rcl_handle,
                    &mut request_id,
                    native_msg.void_ptr_mut(),
                )
            };

            if ret != RCL_RET_OK as i32 {
                println!("action server: could send cancel response. {}", ret);
            }
        }
    }

    fn handle_goal_request(&mut self, server: Arc<Mutex<dyn ActionServer_>>) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Request,
        >::new();
        let ret = unsafe {
            rcl_action_take_goal_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            // this seems normal if client dies.
            return;
        }
        let msg = <<<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Request>::from_native(&request_msg);
        let (uuid_msg, goal) = T::destructure_goal_request_msg(msg);
        let uuid = uuid_msg_to_uuid(&uuid_msg);

        let (cancel_sender, cancel_receiver) = mpsc::channel::<ActionServerCancelRequest>(10);
        self.cancel_senders.insert(uuid, cancel_sender);

        let gr: ActionServerGoalRequest<T> = ActionServerGoalRequest {
            uuid,
            goal,
            cancel_requests: cancel_receiver,
            server: Arc::downgrade(&server),
            request_id: unsafe { request_id.assume_init() },
        };

        // send out request.
        if let Err(e) = self.goal_request_sender.try_send(gr) {
            eprintln!("warning: could not send service request ({})", e)
        }
    }

    fn handle_cancel_request(&mut self) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<action_msgs::srv::CancelGoal::Request>::new();
        let ret = unsafe {
            rcl_action_take_cancel_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };

        let request_id = unsafe { request_id.assume_init() };

        if ret != RCL_RET_OK as i32 {
            // this seems normal if client dies.
            return;
        }

        let mut cancel_response = unsafe { rcl_action_get_zero_initialized_cancel_response() };
        let ret = unsafe {
            rcl_action_process_cancel_request(&self.rcl_handle, &*request_msg, &mut cancel_response)
        };

        if ret != RCL_RET_OK as i32 {
            println!("action server: could not process cancel request. {}", ret);
            return;
        }

        let response_msg =
            action_msgs::srv::CancelGoal::Response::from_native(&cancel_response.msg);

        let return_channels = response_msg
            .goals_canceling
            .iter()
            .flat_map(|goal_info| {
                let uuid = uuid_msg_to_uuid(&goal_info.goal_id);
                self.cancel_senders
                    .get_mut(&uuid)
                    .and_then(|cancel_sender| {
                        let (s, r) = oneshot::channel::<(uuid::Uuid, bool)>();
                        let cr = ActionServerCancelRequest {
                            uuid,
                            response_sender: s,
                        };
                        match cancel_sender.try_send(cr) {
                            Err(_) => {
                                eprintln!("warning: could not send goal cancellation request");
                                None
                            }
                            _ => Some(r),
                        }
                    })
            })
            .collect::<Vec<_>>();

        // because we want to reply to the caller when all goals have been either accepted or rejected,
        // join the channels into one future that we can poll during spin.
        self.active_cancel_requests
            .push((request_id, response_msg, join_all(return_channels)));
    }

    fn handle_goal_expired(&mut self) {
        let mut goal_info = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::new();
        let mut num_expired = 1;

        while num_expired > 1 {
            let ret = unsafe {
                rcl_action_expire_goals(&self.rcl_handle, &mut *goal_info, 1, &mut num_expired)
            };
            if ret != RCL_RET_OK as i32 {
                println!("action server: could not expire goal.");
                return;
            }
            let gi = action_msgs::msg::GoalInfo::from_native(&goal_info);
            let uuid = uuid_msg_to_uuid(&gi.goal_id);
            println!("goal expired: {} - {}", uuid, num_expired);
            // todo
            // self.goals.remove(&uuid);
            self.result_msgs.remove(&uuid);
            self.result_requests.remove(&uuid);
        }
    }

    fn publish_status(&self) {
        unsafe {
            let mut status = rcl_action_get_zero_initialized_goal_status_array();
            let ret = rcl_action_get_goal_status_array(&self.rcl_handle, &mut status);
            if ret != RCL_RET_OK as i32 {
                println!(
                    "action server: failed to get goal status array: {}",
                    Error::from_rcl_error(ret)
                );
                return;
            }
            let ret = rcl_action_publish_status(
                &self.rcl_handle,
                &status as *const _ as *const std::os::raw::c_void,
            );
            if ret != RCL_RET_OK as i32 {
                println!(
                    "action server: failed to publish status: {}",
                    Error::from_rcl_error(ret)
                );
                return;
            }
            rcl_action_goal_status_array_fini(&mut status);
        }
    }

    fn add_goal_handle(&mut self, uuid: uuid::Uuid, goal_handle: *mut rcl_action_goal_handle_t) {
        self.goals.insert(uuid, goal_handle);
    }

    // bit of a hack...
    fn add_result(&mut self, uuid: uuid::Uuid, mut msg: Box<dyn VoidPtr>) {
        // if there are already requests for this goal, send the result immediately.
        if let Some(rr) = self.result_requests.remove(&uuid) {
            for mut req in rr {
                let ret = unsafe {
                    rcl_action_send_result_response(&self.rcl_handle, &mut req, msg.void_ptr_mut())
                };
                if ret != RCL_RET_OK as i32 {
                    println!(
                        "action server: could send result request response. {}",
                        Error::from_rcl_error(ret)
                    );
                }
            }
        }
        self.result_msgs.insert(uuid, msg);
    }

    fn handle_result_request(&mut self) {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Request,
        >::new();
        let ret = unsafe {
            rcl_action_take_result_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            // this seems normal if client dies.
            return;
        }

        let msg = <<<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Request>::from_native(&request_msg);
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: T::destructure_result_request_msg(msg),
            ..action_msgs::msg::GoalInfo::default()
        };
        let goal_info_native = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        // does this goal exist?
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(&self.rcl_handle, &*goal_info_native) };

        let uuid = uuid_msg_to_uuid(&goal_info.goal_id);

        let response_msg = if !goal_exists {
            // Goal does not exists
            println!("goal does not exist :(");
            let status = GoalStatus::Unknown;
            let msg = T::make_result_response_msg(status.to_rcl(), T::Result::default());
            let mut response_msg = WrappedNativeMsg::<
                <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
            >::from(&msg);
            Some(response_msg.void_ptr_mut())
        } else {
            self.result_msgs
                .get_mut(&uuid)
                .map(|msg| msg.void_ptr_mut())
        };

        let mut request_id = unsafe { request_id.assume_init() };
        if let Some(response_msg) = response_msg {
            let ret = unsafe {
                rcl_action_send_result_response(&self.rcl_handle, &mut request_id, response_msg)
            };

            if ret != RCL_RET_OK as i32 {
                println!(
                    "action server: could send result request response. {}",
                    Error::from_rcl_error(ret)
                );
            }
        } else {
            // keep request for later when result comes in
            // todo: add logic that replies to the requests
            self.result_requests
                .entry(uuid)
                .or_insert_with(Vec::new)
                .push(request_id);
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_action_server_fini(&mut self.rcl_handle, node);
            rcl_ros_clock_fini(self.clock_handle.as_mut());
        }
    }
}

/// A handle to an active `Goal`
#[derive(Clone)]
pub struct ActionServerGoal<T>
where
    T: WrappedActionTypeSupport,
{
    pub uuid: uuid::Uuid,
    pub goal: T::Goal,
    server: Weak<Mutex<dyn ActionServer_>>,
}

unsafe impl<T> Send for ActionServerGoal<T> where T: WrappedActionTypeSupport {}

impl<T: 'static> ActionServerGoal<T>
where
    T: WrappedActionTypeSupport,
{
    pub fn is_cancelling(&self) -> Result<bool> {
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;

        let action_server = action_server.lock().unwrap();
        action_server.is_cancelling(&self.uuid)
    }

    pub fn publish_feedback(&self, msg: T::Feedback) -> Result<()>
    where
        T: WrappedActionTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;

        let uuid_msg = unique_identifier_msgs::msg::UUID {
            uuid: self.uuid.as_bytes().to_vec(),
        };
        let feedback_msg = T::make_feedback_msg(uuid_msg, msg);
        let mut native_msg = WrappedNativeMsg::<T::FeedbackMessage>::from(&feedback_msg);
        let ret = unsafe {
            rcl_action_publish_feedback(
                action_server.lock().unwrap().handle(),
                native_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            eprintln!("coult not publish {}", Error::from_rcl_error(ret));
        }
        Ok(()) // todo: error codes
    }

    pub fn cancel(&mut self, msg: T::Result) -> Result<()> {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let mut action_server = action_server.lock().unwrap();

        // todo: check that the goal exists
        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: unique_identifier_msgs::msg::UUID {
                uuid: self.uuid.as_bytes().to_vec(),
            },
            ..action_msgs::msg::GoalInfo::default()
        };
        let goal_info_native = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        // does this goal exist?
        let goal_exists =
            unsafe { rcl_action_server_goal_exists(action_server.handle(), &*goal_info_native) };

        if !goal_exists {
            println!("tried to publish result without a goal");
            return Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID);
        }

        // todo: error handling
        unsafe {
            rcl_action_notify_goal_done(action_server.handle());
        }

        // send out updated statues
        action_server.publish_status();

        // create result message
        let result_msg = T::make_result_response_msg(5, msg); // todo: int8 STATUS_CANCELED  = 5
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid, Box::new(native_msg));

        Ok(())
    }

    pub fn abort(&mut self, msg: T::Result) -> Result<()> {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let mut action_server = action_server.lock().unwrap();

        action_server.set_goal_state(&self.uuid, rcl_action_goal_event_t::GOAL_EVENT_ABORT)?;

        // create result message
        let result_msg = T::make_result_response_msg(6, msg); // todo: int8 STATUS_ABORTED   = 6
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid, Box::new(native_msg));

        Ok(())
    }

    pub fn succeed(&mut self, msg: T::Result) -> Result<()>
    where
        T: WrappedActionTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let mut action_server = action_server.lock().unwrap();

        action_server.set_goal_state(&self.uuid, rcl_action_goal_event_t::GOAL_EVENT_SUCCEED)?;

        // create result message
        let result_msg = T::make_result_response_msg(4, msg); // todo: int8 STATUS_SUCCEEDED = 4
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid, Box::new(native_msg));

        Ok(())
    }
}

pub fn create_action_server_helper(
    node: &mut rcl_node_t,
    action_name: &str,
    clock_handle: *mut rcl_clock_t,
    action_ts: *const rosidl_action_type_support_t,
) -> Result<rcl_action_server_t> {
    let mut server_handle = unsafe { rcl_action_get_zero_initialized_server() };
    let action_name_c_string =
        CString::new(action_name).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let server_options = rcl_action_server_get_default_options();

        rcl_action_server_init(
            &mut server_handle,
            node,
            clock_handle,
            action_ts,
            action_name_c_string.as_ptr(),
            &server_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(server_handle)
    } else {
        Err(Error::from_rcl_error(result))
    }
}

pub fn action_server_get_num_waits(
    rcl_handle: &rcl_action_server_t,
    num_subs: &mut usize,
    num_gc: &mut usize,
    num_timers: &mut usize,
    num_clients: &mut usize,
    num_services: &mut usize,
) -> Result<()> {
    unsafe {
        let result = rcl_action_server_wait_set_get_num_entities(
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
