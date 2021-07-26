include!(concat!(env!("OUT_DIR"), "/_r2r_generated_msgs.rs"));
include!(concat!(
    env!("OUT_DIR"),
    "/_r2r_generated_untyped_helper.rs"
));

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::fmt::Debug;
use std::marker::PhantomData;
use std::mem::MaybeUninit;
use std::ops::{Deref, DerefMut};
use std::time::Duration;

use futures::channel::{mpsc, oneshot};
use futures::future::FutureExt;
use futures::future::TryFutureExt;
use futures::stream::{Stream, StreamExt};
use std::future::Future;

use actions::*;
use msg_gen::*;
use rcl::*;

mod error;
use error::*;

mod utils;
pub use utils::*;

pub type Result<T> = std::result::Result<T, Error>;

pub trait WrappedTypesupport:
    Serialize + serde::de::DeserializeOwned + Default + Debug + Clone
{
    type CStruct;

    fn get_ts() -> &'static rosidl_message_type_support_t;
    fn create_msg() -> *mut Self::CStruct;
    fn destroy_msg(msg: *mut Self::CStruct);
    fn from_native(msg: &Self::CStruct) -> Self;
    fn copy_to_native(&self, msg: &mut Self::CStruct);
}

pub trait WrappedServiceTypeSupport: Debug + Clone {
    type Request: WrappedTypesupport;
    type Response: WrappedTypesupport;

    fn get_ts() -> &'static rosidl_service_type_support_t;
}

pub trait WrappedActionTypeSupport: Debug + Clone {
    type Goal: WrappedTypesupport;
    type Result: WrappedTypesupport;
    type Feedback: WrappedTypesupport;

    // internal...
    type FeedbackMessage: WrappedTypesupport;
    type SendGoal: WrappedServiceTypeSupport;
    type GetResult: WrappedServiceTypeSupport;

    fn get_ts() -> &'static rosidl_action_type_support_t;

    fn make_goal_request_msg(
        goal_id: unique_identifier_msgs::msg::UUID,
        goal: Self::Goal,
    ) -> <<Self as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Request;
    fn make_goal_response_msg(
        accepted: bool,
        stamp: builtin_interfaces::msg::Time,
    ) -> <<Self as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response;
    fn make_feedback_msg(
        goal_id: unique_identifier_msgs::msg::UUID,
        feedback: Self::Feedback,
    ) -> Self::FeedbackMessage;
    fn make_result_request_msg(
        goal_id: unique_identifier_msgs::msg::UUID,
    ) -> <<Self as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Request;
    fn make_result_response_msg(
        status: i8,
        result: Self::Result,
    ) -> <<Self as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response;
    fn destructure_goal_request_msg(
        msg: <<Self as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Request,
    ) -> (unique_identifier_msgs::msg::UUID, Self::Goal);
    fn destructure_goal_response_msg(
        msg: <<Self as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
    ) -> (bool, builtin_interfaces::msg::Time);
    fn destructure_feedback_msg(
        msg: Self::FeedbackMessage,
    ) -> (unique_identifier_msgs::msg::UUID, Self::Feedback);
    fn destructure_result_response_msg(
        msg: <<Self as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
    ) -> (i8, Self::Result);
    fn destructure_result_request_msg(
        msg: <<Self as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Request,
    ) -> unique_identifier_msgs::msg::UUID;
}

#[derive(Debug)]
pub struct WrappedNativeMsg<T>
where
    T: WrappedTypesupport,
{
    pub msg: *mut T::CStruct,
}

pub trait VoidPtr {
    fn void_ptr(&self) -> *const std::os::raw::c_void;
    fn void_ptr_mut(&mut self) -> *mut std::os::raw::c_void;
}

#[derive(Debug)]
pub struct WrappedNativeMsgUntyped {
    ts: &'static rosidl_message_type_support_t,
    msg: *mut std::os::raw::c_void,
    destroy: fn(*mut std::os::raw::c_void),
    msg_to_json: fn(
        native: *const std::os::raw::c_void,
    ) -> std::result::Result<serde_json::Value, serde_json::error::Error>,
    msg_from_json: fn(
        native: *mut std::os::raw::c_void,
        json: serde_json::Value,
    ) -> std::result::Result<(), serde_json::error::Error>,
}

impl WrappedNativeMsgUntyped {
    fn new<T>() -> Self
    where
        T: WrappedTypesupport,
    {
        let destroy = |native: *mut std::os::raw::c_void| {
            let native_msg = native as *mut T::CStruct;
            T::destroy_msg(native_msg);
        };

        let msg_to_json = |native: *const std::os::raw::c_void| {
            let msg = unsafe { T::from_native(&*(native as *const T::CStruct)) };
            serde_json::to_value(&msg)
        };

        let msg_from_json = |native: *mut std::os::raw::c_void, json: serde_json::Value| {
            serde_json::from_value(json).map(|msg: T| unsafe {
                msg.copy_to_native(&mut *(native as *mut T::CStruct));
            })
        };

        WrappedNativeMsgUntyped {
            ts: T::get_ts(),
            msg: T::create_msg() as *mut std::os::raw::c_void,
            destroy,
            msg_to_json,
            msg_from_json,
        }
    }

    fn to_json(&self) -> Result<serde_json::Value> {
        let json = (self.msg_to_json)(self.msg);
        json.map_err(|serde_err| Error::SerdeError {
            err: serde_err.to_string(),
        })
    }

    fn from_json(&mut self, json: serde_json::Value) -> Result<()> {
        (self.msg_from_json)(self.msg, json).map_err(|serde_err| Error::SerdeError {
            err: serde_err.to_string(),
        })
    }
}

impl VoidPtr for WrappedNativeMsgUntyped {
    fn void_ptr(&self) -> *const std::os::raw::c_void {
        self.msg as *const _ as *const std::os::raw::c_void
    }

    fn void_ptr_mut(&mut self) -> *mut std::os::raw::c_void {
        self.msg as *mut _ as *mut std::os::raw::c_void
    }
}

impl Drop for WrappedNativeMsgUntyped {
    fn drop(&mut self) {
        (self.destroy)(self.msg);
    }
}

impl<T> WrappedNativeMsg<T>
where
    T: WrappedTypesupport,
{
    pub fn new() -> Self {
        WrappedNativeMsg {
            msg: T::create_msg(),
        }
    }

    pub fn from(msg: &T) -> Self {
        let mut native_msg = Self::new();
        msg.copy_to_native(&mut native_msg);
        native_msg
    }
}

impl<T: 'static> VoidPtr for WrappedNativeMsg<T>
where
    T: WrappedTypesupport,
{
    fn void_ptr(&self) -> *const std::os::raw::c_void {
        self.msg as *const _ as *const std::os::raw::c_void
    }

    fn void_ptr_mut(&mut self) -> *mut std::os::raw::c_void {
        self.msg as *mut _ as *mut std::os::raw::c_void
    }
}

impl<T> Drop for WrappedNativeMsg<T>
where
    T: WrappedTypesupport,
{
    fn drop(&mut self) {
        T::destroy_msg(self.msg);
    }
}

impl<T> Deref for WrappedNativeMsg<T>
where
    T: WrappedTypesupport,
{
    type Target = T::CStruct;

    fn deref(&self) -> &Self::Target {
        unsafe { &(*self.msg) }
    }
}

impl<T> DerefMut for WrappedNativeMsg<T>
where
    T: WrappedTypesupport,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut (*self.msg) }
    }
}

pub trait Sub {
    fn handle(&self) -> &rcl_subscription_t;
    fn handle_incoming(&mut self) -> ();
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();
}

struct WrappedSub<T>
where
    T: WrappedTypesupport,
{
    rcl_handle: rcl_subscription_t,
    sender: mpsc::Sender<T>,
}

struct WrappedSubNative<T>
where
    T: WrappedTypesupport,
{
    rcl_handle: rcl_subscription_t,
    sender: mpsc::Sender<WrappedNativeMsg<T>>,
}

struct WrappedSubUntyped {
    rcl_handle: rcl_subscription_t,
    topic_type: String,
    sender: mpsc::Sender<Result<serde_json::Value>>,
}

impl<T: 'static> Sub for WrappedSub<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> () {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsg::<T>::new();
        let ret = unsafe {
            rcl_take(
                &self.rcl_handle,
                msg.void_ptr_mut(),
                &mut msg_info,
                std::ptr::null_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let msg = T::from_native(&msg);
            match self.sender.try_send(msg) {
                Err(e) => println!("error {:?}", e),
                _ => (),
            }
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl<T: 'static> Sub for WrappedSubNative<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> () {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsg::<T>::new();
        let ret = unsafe {
            rcl_take(
                &self.rcl_handle,
                msg.void_ptr_mut(),
                &mut msg_info,
                std::ptr::null_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            match self.sender.try_send(msg) {
                Err(e) => println!("error {:?}", e),
                _ => (),
            }
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl Sub for WrappedSubUntyped {
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> () {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsgUntyped::new_from(&self.topic_type)
            .expect(&format!("no typesupport for {}", self.topic_type));
        let ret = unsafe {
            rcl_take(
                &self.rcl_handle,
                msg.void_ptr_mut(),
                &mut msg_info,
                std::ptr::null_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let json = msg.to_json();
            match self.sender.try_send(json) {
                Err(e) => println!("error {:?}", e),
                _ => (),
            }
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

// services
struct WrappedService<T>
where
    T: WrappedServiceTypeSupport,
{
    rcl_handle: rcl_service_t,
    rcl_request: rmw_request_id_t,
    callback: Box<dyn FnMut(T::Request) -> T::Response>,
    rcl_request_msg: WrappedNativeMsg<T::Request>,
}

pub trait Service {
    fn handle(&self) -> &rcl_service_t;
    fn run_cb(&mut self) -> ();
    fn rcl_request_id(&mut self) -> *mut rmw_request_id_t;
    fn rcl_request_msg(&mut self) -> *mut std::os::raw::c_void;
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();
}

impl<T: 'static> Service for WrappedService<T>
where
    T: WrappedServiceTypeSupport,
{
    fn handle(&self) -> &rcl_service_t {
        &self.rcl_handle
    }

    fn rcl_request_msg(&mut self) -> *mut std::os::raw::c_void {
        self.rcl_request_msg.void_ptr_mut()
    }

    fn rcl_request_id(&mut self) -> *mut rmw_request_id_t {
        &mut self.rcl_request
    }

    fn run_cb(&mut self) -> () {
        let request = T::Request::from_native(&self.rcl_request_msg);
        let response = (self.callback)(request);
        let mut native_response = WrappedNativeMsg::<T::Response>::from(&response);
        let res = unsafe {
            rcl_send_response(
                &self.rcl_handle,
                &mut self.rcl_request,
                native_response.void_ptr_mut(),
            )
        };

        // TODO
        if res != RCL_RET_OK as i32 {
            eprintln!("service error {}", res);
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_service_fini(&mut self.rcl_handle, node);
        }
    }
}

// client
struct WrappedClient<T>
where
    T: WrappedServiceTypeSupport,
{
    rcl_handle: rcl_client_t,
    // store callbacks with request sequence id and callback function
    response_channels: Vec<(i64, oneshot::Sender<T::Response>)>,
}

pub trait Client_ {
    fn handle(&self) -> &rcl_client_t;
    fn handle_response(&mut self) -> ();
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();
}

impl<T: 'static> Client_ for WrappedClient<T>
where
    T: WrappedServiceTypeSupport,
{
    fn handle(&self) -> &rcl_client_t {
        &self.rcl_handle
    }

    fn handle_response(&mut self) -> () {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut response_msg = WrappedNativeMsg::<T::Response>::new();

        let ret = unsafe {
            rcl_take_response(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                response_msg.void_ptr_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            let request_id = unsafe { request_id.assume_init() };
            if let Some(idx) = self
                .response_channels
                .iter()
                .position(|(id, _)| id == &request_id.sequence_number)
            {
                let (_, sender) = self.response_channels.swap_remove(idx);
                let response = T::Response::from_native(&response_msg);
                match sender.send(response) {
                    Ok(()) => {}
                    Err(e) => {
                        println!("error sending to client: {:?}", e);
                    }
                }
            } else {
                let we_have: String = self
                    .response_channels
                    .iter()
                    .map(|(id, _)| id.to_string())
                    .collect::<Vec<_>>()
                    .join(",");
                eprintln!(
                    "no such req id: {}, we have [{}], ignoring",
                    request_id.sequence_number, we_have
                );
            }
        } // TODO handle failure.
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_client_fini(&mut self.rcl_handle, node);
        }
    }
}

// action clients
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum GoalStatus {
    Unknown,
    Accepted,
    Executing,
    Canceling,
    Succeeded,
    Canceled,
    Aborted,
}

impl GoalStatus {
    #[allow(dead_code)]
    pub fn to_rcl(&self) -> i8 {
        match self {
            GoalStatus::Unknown => 0,
            GoalStatus::Accepted => 1,
            GoalStatus::Executing => 2,
            GoalStatus::Canceling => 3,
            GoalStatus::Succeeded => 4,
            GoalStatus::Canceled => 5,
            GoalStatus::Aborted => 6,
        }
    }

    pub fn from_rcl(s: i8) -> Self {
        match s {
            0 => GoalStatus::Unknown,
            1 => GoalStatus::Accepted,
            2 => GoalStatus::Executing,
            3 => GoalStatus::Canceling,
            4 => GoalStatus::Succeeded,
            5 => GoalStatus::Canceled,
            6 => GoalStatus::Aborted,
            _ => panic!("unknown action status: {}", s),
        }
    }
}

struct WrappedActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    rcl_handle: rcl_action_client_t,
    goal_response_channels: Vec<(
        i64,
        oneshot::Sender<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
        >,
    )>,
    cancel_response_channels: Vec<(i64, oneshot::Sender<action_msgs::srv::CancelGoal::Response>)>,
    feedback_senders: Vec<(uuid::Uuid, mpsc::Sender<T::Feedback>)>,
    result_requests: Vec<(i64, uuid::Uuid)>,
    result_senders: Vec<(uuid::Uuid, oneshot::Sender<T::Result>)>,
    goal_status: HashMap<uuid::Uuid, GoalStatus>,
}

pub trait ActionClient_ {
    fn handle(&self) -> &rcl_action_client_t;
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();

    fn handle_goal_response(&mut self) -> ();
    fn handle_cancel_response(&mut self) -> ();
    fn handle_feedback_msg(&mut self) -> ();
    fn handle_status_msg(&mut self) -> ();
    fn handle_result_response(&mut self) -> ();

    fn send_result_request(&mut self, uuid: uuid::Uuid) -> ();
}

use std::convert::TryInto;
fn vec_to_uuid_bytes<T>(v: Vec<T>) -> [T; 16] {
    v.try_into().unwrap_or_else(|v: Vec<T>| {
        panic!("Expected a Vec of length {} but it was {}", 16, v.len())
    })
}

impl<T> WrappedActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    fn get_goal_status(&self, uuid: &uuid::Uuid) -> GoalStatus {
        *self.goal_status.get(uuid).unwrap_or(&GoalStatus::Unknown)
    }

    fn send_cancel_request(&mut self, goal: &uuid::Uuid) -> Result<impl Future<Output = Result<()>>>
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

    fn handle_goal_response(&mut self) -> () {
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
                match sender.send(response) {
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

    fn handle_cancel_response(&mut self) -> () {
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
                match sender.send(response) {
                    Err(e) => eprintln!("warning: could not send cancel response msg ({:?})", e),
                    _ => (),
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

    fn handle_feedback_msg(&mut self) -> () {
        let mut feedback_msg = WrappedNativeMsg::<T::FeedbackMessage>::new();
        let ret =
            unsafe { rcl_action_take_feedback(&self.rcl_handle, feedback_msg.void_ptr_mut()) };
        if ret == RCL_RET_OK as i32 {
            let msg = T::FeedbackMessage::from_native(&feedback_msg);
            let (uuid, feedback) = T::destructure_feedback_msg(msg);
            let msg_uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(uuid.uuid));
            if let Some((_, sender)) = self
                .feedback_senders
                .iter_mut()
                .find(|(uuid, _)| uuid == &msg_uuid)
            {
                match sender.try_send(feedback) {
                    Err(e) => eprintln!("warning: could not send feedback msg ({})", e),
                    _ => (),
                }
            }
        }
    }

    fn handle_status_msg(&mut self) -> () {
        let mut status_array = WrappedNativeMsg::<action_msgs::msg::GoalStatusArray>::new();
        let ret = unsafe { rcl_action_take_status(&self.rcl_handle, status_array.void_ptr_mut()) };
        if ret == RCL_RET_OK as i32 {
            let arr = action_msgs::msg::GoalStatusArray::from_native(&status_array);
            for a in &arr.status_list {
                let uuid =
                    uuid::Uuid::from_bytes(vec_to_uuid_bytes(a.goal_info.goal_id.uuid.clone()));
                if !self.result_senders.iter().any(|(suuid, _)| suuid == &uuid) {
                    continue;
                }
                let status = GoalStatus::from_rcl(a.status);
                *self.goal_status.entry(uuid).or_insert(GoalStatus::Unknown) = status;
            }
        }
    }

    fn handle_result_response(&mut self) -> () {
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
                    if status != GoalStatus::Succeeded {
                        println!("goal status failed: {:?}, result: {:?}", status, result);
                        // this will drop the sender which makes the receiver fail with "canceled"
                    } else {
                        match sender.send(result) {
                            Ok(()) => {}
                            Err(e) => {
                                println!("error sending result to action client: {:?}", e);
                            }
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

    fn send_result_request(&mut self, uuid: uuid::Uuid) -> () {
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

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_action_client_fini(&mut self.rcl_handle, node);
        }
    }
}

// action servers
struct WrappedActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    rcl_handle: rcl_action_server_t,
    clock_handle: Box<rcl_clock_t>,
    accept_goal_cb: Box<dyn FnMut(&uuid::Uuid, &T::Goal) -> bool>,
    accept_cancel_cb: Box<dyn FnMut(&ServerGoal<T>) -> bool>,
    goal_cb: Box<dyn FnMut(ServerGoal<T>)>,
    goals: HashMap<uuid::Uuid, ServerGoal<T>>,
    result_msgs: HashMap<uuid::Uuid, Box<dyn VoidPtr>>,
    result_requests: HashMap<uuid::Uuid, Vec<rmw_request_id_t>>,
}

pub trait ActionServer_ {
    fn handle(&self) -> &rcl_action_server_t;
    fn handle_goal_request(&mut self, server: Arc<Mutex<dyn ActionServer_>>) -> ();
    fn handle_cancel_request(&mut self) -> ();
    fn handle_result_request(&mut self) -> ();
    fn handle_goal_expired(&mut self) -> ();
    fn publish_status(&self) -> ();
    fn add_result(&mut self, uuid: uuid::Uuid, msg: Box<dyn VoidPtr>) -> ();
    fn destroy(&mut self, node: &mut rcl_node_t);
}

impl<T: 'static> ActionServer_ for WrappedActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    fn handle(&self) -> &rcl_action_server_t {
        &self.rcl_handle
    }

    fn handle_goal_request(&mut self, server: Arc<Mutex<dyn ActionServer_>>) -> () {
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
        let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(uuid_msg.uuid.clone()));
        let goal_accepted = (self.accept_goal_cb)(&uuid, &goal);
        let time = builtin_interfaces::msg::Time::default();

        let goal_info = action_msgs::msg::GoalInfo {
            goal_id: uuid_msg,
            stamp: time.clone(),
        };

        let native_goal_info = WrappedNativeMsg::<action_msgs::msg::GoalInfo>::from(&goal_info);

        let goal_handle: Option<*mut rcl_action_goal_handle_t> = if goal_accepted {
            unsafe {
                Some(rcl_action_accept_new_goal(
                    &mut self.rcl_handle,
                    &*native_goal_info,
                ))
            }
        } else {
            None
        };

        // send response
        let response_msg = T::make_goal_response_msg(goal_accepted, time);
        let mut response_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
        >::from(&response_msg);

        let ret = unsafe {
            let mut request_id = request_id.assume_init();
            rcl_action_send_goal_response(
                &self.rcl_handle,
                &mut request_id,
                response_msg.void_ptr_mut(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            println!("action server: failed to send goal response");
            return;
        }

        // if we accepted the goal, update its state machine and publish all goal statuses
        if let Some(goal_handle) = goal_handle {
            unsafe {
                rcl_action_update_goal_state(
                    goal_handle,
                    rcl_action_goal_event_t::GOAL_EVENT_EXECUTE,
                );
            }

            self.publish_status();

            // run the user supplied cb with newly created goal handle object
            let g: ServerGoal<T> = ServerGoal {
                uuid,
                goal,
                handle: Arc::new(Mutex::new(goal_handle)),
                server: Arc::downgrade(&server),
            };

            self.goals.insert(uuid, g.clone());

            // start goal callback
            (self.goal_cb)(g);
        }
    }

    fn handle_cancel_request(&mut self) -> () {
        let mut request_id = MaybeUninit::<rmw_request_id_t>::uninit();
        let mut request_msg = WrappedNativeMsg::<action_msgs::srv::CancelGoal::Request>::new();
        let ret = unsafe {
            rcl_action_take_cancel_request(
                &self.rcl_handle,
                request_id.as_mut_ptr(),
                request_msg.void_ptr_mut(),
            )
        };

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

        let mut response_msg =
            action_msgs::srv::CancelGoal::Response::from_native(&cancel_response.msg);

        // let user filter cancelled goals.
        let requested_cancels = response_msg.goals_canceling.len();
        response_msg.goals_canceling.retain(|goal_info| {
            let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(goal_info.goal_id.uuid.clone()));
            if let Some(goal) = self.goals.get(&uuid) {
                (self.accept_cancel_cb)(goal)
            } else {
                true
            }
        });

        response_msg.goals_canceling.iter().for_each(|goal_info| {
            let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(goal_info.goal_id.uuid.clone()));
            if let Some(goal) = self.goals.get_mut(&uuid) {
                goal.set_cancel();
            }
        });

        // check if all cancels were rejected.
        if requested_cancels >= 1 && response_msg.goals_canceling.is_empty() {
            response_msg.return_code = 1; // TODO: auto generate these (int8 ERROR_REJECTED=1)
        }

        if !response_msg.goals_canceling.is_empty() {
            // at least one goal state changed, publish a new status message
            self.publish_status();
        }

        let mut native_msg =
            WrappedNativeMsg::<action_msgs::srv::CancelGoal::Response>::from(&response_msg);
        let ret = unsafe {
            let mut request_id = request_id.assume_init();
            rcl_action_send_cancel_response(
                &self.rcl_handle,
                &mut request_id,
                native_msg.void_ptr_mut(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            println!("action server: could send cancel response. {}", ret);
            return;
        }
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
            let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(gi.goal_id.uuid.clone()));
            println!("goal expired: {} - {}", uuid, num_expired);
            self.goals.remove(&uuid);
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

    // bit of a hack...
    fn add_result(&mut self, uuid: uuid::Uuid, mut msg: Box<dyn VoidPtr>) -> () {
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

    fn handle_result_request(&mut self) -> () {
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

        let uuid = uuid::Uuid::from_bytes(vec_to_uuid_bytes(goal_info.goal_id.uuid.clone()));

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
                return;
            }
        } else {
            // keep request for later when result comes in
            // todo: add logic that replies to the requests
            self.result_requests
                .entry(uuid)
                .or_insert(vec![])
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

// The publish function is thread safe. ROS2 docs state:
// =============
//
// This function is thread safe so long as access to both the
// publisher and the" `ros_message` is synchronized."  That means that
// calling rcl_publish() from multiple threads is allowed, but"
// calling rcl_publish() at the same time as non-thread safe
// publisher" functions is not, e.g. calling rcl_publish() and
// rcl_publisher_fini()" concurrently is not allowed."  Before calling
// rcl_publish() the message can change and after calling"
// rcl_publish() the message can change, but it cannot be changed
// during the" publish call."  The same `ros_message`, however, can be
// passed to multiple calls of" rcl_publish() simultaneously, even if
// the publishers differ."  The `ros_message` is unmodified by
// rcl_publish()."
//
// TODO: I guess there is a potential error source in destructuring
// while calling publish. I don't think its worth to protect with a
// mutex/rwlock for this though...
//
// Methods that mutate need to called from the thread owning the Node.
// I don't think we can count on Node being generally thread-safe.
// So keep pub/sub management and polling contained to one thread
// and send out publishers.

unsafe impl<T> Send for Publisher<T> where T: WrappedTypesupport {}

use std::sync::{Arc, Mutex, Weak};
#[derive(Debug, Clone)]
pub struct Publisher<T>
where
    T: WrappedTypesupport,
{
    handle: Weak<rcl_publisher_t>,
    type_: PhantomData<T>,
}

unsafe impl Send for PublisherUntyped {}
#[derive(Debug, Clone)]
pub struct PublisherUntyped {
    handle: Weak<rcl_publisher_t>,
    type_: String,
}

// Same reasoning for clients.
unsafe impl<T> Send for Client<T> where T: WrappedServiceTypeSupport {}

pub struct Client<T>
where
    T: WrappedServiceTypeSupport,
{
    client_: Weak<Mutex<WrappedClient<T>>>,
}

unsafe impl<T> Send for ActionClient<T> where T: WrappedActionTypeSupport {}

#[derive(Clone)]
pub struct ActionClient<T>
where
    T: WrappedActionTypeSupport,
{
    client: Weak<Mutex<WrappedActionClient<T>>>,
}

#[derive(Clone)]
pub struct ClientGoal<T>
where
    T: WrappedActionTypeSupport,
{
    client: Weak<Mutex<WrappedActionClient<T>>>,
    pub uuid: uuid::Uuid,
    feedback: Arc<Mutex<Option<mpsc::Receiver<T::Feedback>>>>,
    result: Arc<Mutex<Option<oneshot::Receiver<T::Result>>>>,
}

#[derive(Clone)]
pub struct ServerGoal<T>
where
    T: WrappedActionTypeSupport,
{
    pub uuid: uuid::Uuid,
    pub goal: T::Goal,
    handle: Arc<Mutex<*mut rcl_action_goal_handle_t>>,
    server: Weak<Mutex<dyn ActionServer_>>,
}

unsafe impl<T> Send for ServerGoal<T> where T: WrappedActionTypeSupport {}

impl<T: 'static> ServerGoal<T>
where
    T: WrappedActionTypeSupport,
{
    pub fn is_cancelling(&self) -> bool {
        let mut state = 0u8; // TODO: int8 STATUS_UNKNOWN   = 0;
        let ret = unsafe {
            let handle = self.handle.lock().unwrap();
            rcl_action_goal_handle_get_status(*handle, &mut state)
        };

        if ret != RCL_RET_OK as i32 {
            println!("action server: Failed to get goal handle state: {}", ret);
        }
        return state == 3u8; // TODO: int8 STATUS_CANCELING
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

        if ret == RCL_RET_OK as i32 {
            Ok(())
        } else {
            eprintln!("coult not publish {}", Error::from_rcl_error(ret));
            Ok(()) // todo: error codes
        }
    }

    fn set_cancel(&mut self) {
        let ret = unsafe {
            let handle = self.handle.lock().unwrap();
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_t::GOAL_EVENT_CANCEL_GOAL)
        };

        if ret != RCL_RET_OK as i32 {
            println!(
                "action server: could not cancel goal: {}",
                Error::from_rcl_error(ret)
            );
        }
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
        action_server.add_result(self.uuid.clone(), Box::new(native_msg));

        Ok(())
    }

    pub fn abort(&mut self, msg: T::Result) -> Result<()> {
        // todo: error handling
        let ret = unsafe {
            let handle = self.handle.lock().unwrap();
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_t::GOAL_EVENT_ABORT)
        };

        if ret != RCL_RET_OK as i32 {
            println!(
                "action server: could not cancel goal: {}",
                Error::from_rcl_error(ret)
            );
        }

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
            println!("tried to abort without a goal");
            return Err(Error::RCL_RET_ACTION_GOAL_HANDLE_INVALID);
        }

        // todo: error handling
        unsafe {
            rcl_action_notify_goal_done(action_server.handle());
        }

        // send out updated statues
        action_server.publish_status();

        // create result message
        let result_msg = T::make_result_response_msg(6, msg); // todo: int8 STATUS_ABORTED   = 6
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid.clone(), Box::new(native_msg));

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
            let handle = self.handle.lock().unwrap();
            rcl_action_update_goal_state(*handle, rcl_action_goal_event_t::GOAL_EVENT_SUCCEED);
        }

        // todo: error handling
        unsafe {
            rcl_action_notify_goal_done(action_server.handle());
        }

        // send out updated statues
        action_server.publish_status();

        // create result message
        let result_msg = T::make_result_response_msg(4, msg); // todo: int8 STATUS_SUCCEEDED = 4
        let native_msg = WrappedNativeMsg::<
            <<T as WrappedActionTypeSupport>::GetResult as WrappedServiceTypeSupport>::Response,
        >::from(&result_msg);
        action_server.add_result(self.uuid.clone(), Box::new(native_msg));

        Ok(())
    }
}

#[derive(Clone)]
pub struct ActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    server: Weak<Mutex<WrappedActionServer<T>>>,
}

impl<T> ActionServer<T>
where
    T: WrappedActionTypeSupport,
{
    pub fn is_valid(&self) -> Result<bool> {
        // upgrade to actual ref. if still alive
        let action_server = self
            .server
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_SERVER_INVALID)?;
        let action_server = action_server.lock().unwrap();

        Ok(unsafe { rcl_action_server_is_valid(&action_server.rcl_handle) })
    }
}

#[derive(Debug, Clone)]
pub struct Context {
    context_handle: Arc<Mutex<ContextHandle>>,
}

// Not 100% about this one. From our end the context is rarely used
// and can be locked by a mutex for that. But I haven't investigated
// if its use is thread-safe between nodes. May remove send here
// later.
unsafe impl Send for Context {}

impl Context {
    pub fn create() -> Result<Context> {
        let mut ctx: Box<rcl_context_t> = unsafe { Box::new(rcl_get_zero_initialized_context()) };
        // argc/v
        let args = std::env::args()
            .map(|arg| CString::new(arg).unwrap())
            .collect::<Vec<CString>>();
        let mut c_args = args
            .iter()
            .map(|arg| arg.as_ptr())
            .collect::<Vec<*const ::std::os::raw::c_char>>();
        c_args.push(std::ptr::null());

        let is_valid = unsafe {
            let allocator = rcutils_get_default_allocator();
            let mut init_options = rcl_get_zero_initialized_init_options();
            rcl_init_options_init(&mut init_options, allocator);
            rcl_init(
                (c_args.len() - 1) as ::std::os::raw::c_int,
                c_args.as_ptr(),
                &init_options,
                ctx.as_mut(),
            );
            rcl_init_options_fini(&mut init_options as *mut _);
            rcl_context_is_valid(ctx.as_mut())
        };

        let logging_ok = unsafe {
            let _guard = log_guard();
            let ret = rcl_logging_configure(
                &ctx.as_ref().global_arguments,
                &rcutils_get_default_allocator(),
            );
            ret == RCL_RET_OK as i32
        };

        if is_valid && logging_ok {
            Ok(Context {
                context_handle: Arc::new(Mutex::new(ContextHandle(ctx))),
            })
        } else {
            Err(Error::RCL_RET_ERROR) // TODO
        }
    }

    pub fn is_valid(&self) -> bool {
        let mut ctx = self.context_handle.lock().unwrap();
        unsafe { rcl_context_is_valid(ctx.as_mut()) }
    }
}

#[derive(Debug)]
pub struct ContextHandle(Box<rcl_context_t>);

impl Deref for ContextHandle {
    type Target = Box<rcl_context_t>;

    fn deref<'a>(&'a self) -> &'a Box<rcl_context_t> {
        &self.0
    }
}

impl DerefMut for ContextHandle {
    fn deref_mut<'a>(&'a mut self) -> &'a mut Box<rcl_context_t> {
        &mut self.0
    }
}

impl Drop for ContextHandle {
    fn drop(&mut self) {
        // TODO: error handling? atleast probably need rcl_reset_error
        unsafe {
            rcl::rcl_shutdown(self.0.as_mut());
            rcl::rcl_context_fini(self.0.as_mut());
        }
    }
}

#[derive(Debug)]
pub enum ParameterValue {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    ByteArray(Vec<u8>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl ParameterValue {
    fn from_rcl(v: &rcl_variant_t) -> Self {
        if v.bool_value != std::ptr::null_mut() {
            ParameterValue::Bool(unsafe { *v.bool_value })
        } else if v.integer_value != std::ptr::null_mut() {
            ParameterValue::Integer(unsafe { *v.integer_value })
        } else if v.double_value != std::ptr::null_mut() {
            ParameterValue::Double(unsafe { *v.double_value })
        } else if v.string_value != std::ptr::null_mut() {
            let s = unsafe { CStr::from_ptr(v.string_value) };
            let string = s.to_str().unwrap_or("").to_owned();
            ParameterValue::String(string)
        } else if v.byte_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts((*v.byte_array_value).values, (*v.byte_array_value).size)
            };
            ParameterValue::ByteArray(vals.iter().cloned().collect())
        } else if v.bool_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts((*v.bool_array_value).values, (*v.bool_array_value).size)
            };
            ParameterValue::BoolArray(vals.iter().cloned().collect())
        } else if v.integer_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.integer_array_value).values,
                    (*v.integer_array_value).size,
                )
            };
            ParameterValue::IntegerArray(vals.iter().cloned().collect())
        } else if v.double_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.double_array_value).values,
                    (*v.double_array_value).size,
                )
            };
            ParameterValue::DoubleArray(vals.iter().cloned().collect())
        } else if v.string_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.string_array_value).data,
                    (*v.string_array_value).size,
                )
            };
            let s = vals
                .iter()
                .map(|cs| {
                    let s = unsafe { CStr::from_ptr(*cs) };
                    s.to_str().unwrap_or("").to_owned()
                })
                .collect();
            ParameterValue::StringArray(s)
        } else {
            ParameterValue::NotSet
        }
    }
}

pub struct Node {
    context: Context,
    pub params: HashMap<String, ParameterValue>,
    node_handle: Box<rcl_node_t>,
    // the node owns the subscribers
    subs: Vec<Box<dyn Sub>>,
    // services,
    services: Vec<Box<dyn Service>>,
    // clients with hack to avoid locking just to wait...,
    clients: Vec<(rcl_client_t, Arc<Mutex<dyn Client_>>)>,
    // action clients
    action_clients: Vec<(rcl_action_client_t, Arc<Mutex<dyn ActionClient_>>)>,
    // action servers
    action_servers: Vec<(rcl_action_server_t, Arc<Mutex<dyn ActionServer_>>)>,
    // timers,
    timers: Vec<Timer_>,
    // and the publishers, whom we allow to be shared.. hmm.
    pubs: Vec<Arc<rcl_publisher_t>>,
}

unsafe impl Send for Node {}

impl Node {
    pub fn name(&self) -> Result<String> {
        let cstr = unsafe { rcl_node_get_name(self.node_handle.as_ref()) };
        if cstr == std::ptr::null() {
            return Err(Error::RCL_RET_NODE_INVALID);
        }
        let s = unsafe { CStr::from_ptr(cstr) };
        Ok(s.to_str().unwrap_or("").to_owned())
    }

    pub fn fully_qualified_name(&self) -> Result<String> {
        let cstr = unsafe { rcl_node_get_fully_qualified_name(self.node_handle.as_ref()) };
        if cstr == std::ptr::null() {
            return Err(Error::RCL_RET_NODE_INVALID);
        }
        let s = unsafe { CStr::from_ptr(cstr) };
        Ok(s.to_str().unwrap_or("").to_owned())
    }

    pub fn namespace(&self) -> Result<String> {
        let cstr = unsafe { rcl_node_get_namespace(self.node_handle.as_ref()) };
        if cstr == std::ptr::null() {
            return Err(Error::RCL_RET_NODE_INVALID);
        }
        let s = unsafe { CStr::from_ptr(cstr) };
        Ok(s.to_str().unwrap_or("").to_owned())
    }

    fn load_params(&mut self) -> Result<()> {
        let ctx = self.context.context_handle.lock().unwrap();
        let mut params: Box<*mut rcl_params_t> = Box::new(std::ptr::null_mut());

        let ret =
            unsafe { rcl_arguments_get_param_overrides(&ctx.global_arguments, params.as_mut()) };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not read parameters: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        if *params == std::ptr::null_mut() {
            return Ok(());
        }

        let node_names = unsafe {
            std::slice::from_raw_parts(
                (*(*params.as_ref())).node_names,
                (*(*params.as_ref())).num_nodes,
            )
        };

        let node_params = unsafe {
            std::slice::from_raw_parts(
                (*(*params.as_ref())).params,
                (*(*params.as_ref())).num_nodes,
            )
        };

        let qualified_name = self.fully_qualified_name()?;
        let name = self.name()?;

        for (nn, np) in node_names.iter().zip(node_params) {
            let node_name_cstr = unsafe { CStr::from_ptr(*nn) };
            let node_name = node_name_cstr.to_str().unwrap_or("");

            // This is copied from rclcpp, but there is a comment there suggesting
            // that more wildcards will be added in the future. Take note and mimic
            // their behavior.
            if !(node_name == "/**"
                || node_name == "**"
                || qualified_name == node_name
                || name == node_name)
            {
                continue;
            }

            // make key value pairs.
            let param_names =
                unsafe { std::slice::from_raw_parts(np.parameter_names, np.num_params) };

            let param_values =
                unsafe { std::slice::from_raw_parts(np.parameter_values, np.num_params) };

            for (s, v) in param_names.iter().zip(param_values) {
                let s = unsafe { CStr::from_ptr(*s) };
                let key = s.to_str().unwrap_or("");
                let val = ParameterValue::from_rcl(&*v);
                self.params.insert(key.to_owned(), val);
            }
        }

        unsafe { rcl_yaml_node_struct_fini(*params) };
        Ok(())
    }

    pub fn create(ctx: Context, name: &str, namespace: &str) -> Result<Node> {
        // cleanup default options.
        let (res, node_handle) = {
            let mut ctx_handle = ctx.context_handle.lock().unwrap();

            let c_node_name = CString::new(name).unwrap();
            let c_node_ns = CString::new(namespace).unwrap();
            let mut node_handle: Box<rcl_node_t> =
                unsafe { Box::new(rcl_get_zero_initialized_node()) };
            let res = unsafe {
                let node_options = rcl_node_get_default_options();
                rcl_node_init(
                    node_handle.as_mut(),
                    c_node_name.as_ptr(),
                    c_node_ns.as_ptr(),
                    ctx_handle.as_mut(),
                    &node_options as *const _,
                )
            };
            (res, node_handle)
        };

        if res == RCL_RET_OK as i32 {
            let mut node = Node {
                params: HashMap::new(),
                context: ctx,
                node_handle: node_handle,
                subs: Vec::new(),
                services: Vec::new(),
                clients: Vec::new(),
                action_clients: Vec::new(),
                action_servers: Vec::new(),
                timers: Vec::new(),
                pubs: Vec::new(),
            };
            node.load_params()?;
            Ok(node)
        } else {
            eprintln!("could not create node{}", res);
            Err(Error::from_rcl_error(res))
        }
    }

    fn create_subscription_helper(
        &mut self,
        topic: &str,
        ts: *const rosidl_message_type_support_t,
    ) -> Result<rcl_subscription_t> {
        let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
        let topic_c_string = CString::new(topic).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

        let result = unsafe {
            let mut subscription_options = rcl_subscription_get_default_options();
            subscription_options.qos = rmw_qos_profile_t::default();
            rcl_subscription_init(
                &mut subscription_handle,
                self.node_handle.as_mut(),
                ts,
                topic_c_string.as_ptr(),
                &subscription_options,
            )
        };
        if result == RCL_RET_OK as i32 {
            Ok(subscription_handle)
        } else {
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn subscribe<T: 'static>(&mut self, topic: &str) -> Result<impl Stream<Item = T> + Unpin>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle = self.create_subscription_helper(topic, T::get_ts())?;
        let (sender, receiver) = mpsc::channel::<T>(10);

        let ws = WrappedSub {
            rcl_handle: subscription_handle,
            sender,
        };
        self.subs.push(Box::new(ws));
        Ok(receiver)
    }

    pub fn subscribe_native<T: 'static>(
        &mut self,
        topic: &str,
    ) -> Result<impl Stream<Item = WrappedNativeMsg<T>>>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle = self.create_subscription_helper(topic, T::get_ts())?;
        let (sender, receiver) = mpsc::channel::<WrappedNativeMsg<T>>(10);

        let ws = WrappedSubNative {
            rcl_handle: subscription_handle,
            sender,
        };
        self.subs.push(Box::new(ws));
        Ok(receiver)
    }

    // Its not really untyped since we know the underlying type... But we throw this info away :)
    pub fn subscribe_untyped(
        &mut self,
        topic: &str,
        topic_type: &str,
    ) -> Result<impl Stream<Item = Result<serde_json::Value>>> {
        let msg = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let subscription_handle = self.create_subscription_helper(topic, msg.ts)?;
        let (sender, receiver) = mpsc::channel::<Result<serde_json::Value>>(10);

        let ws = WrappedSubUntyped {
            rcl_handle: subscription_handle,
            topic_type: topic_type.to_string(),
            sender,
        };
        self.subs.push(Box::new(ws));
        Ok(receiver)
    }

    pub fn create_service_helper(
        &mut self,
        service_name: &str,
        service_ts: *const rosidl_service_type_support_t,
    ) -> Result<rcl_service_t> {
        let mut service_handle = unsafe { rcl_get_zero_initialized_service() };
        let service_name_c_string =
            CString::new(service_name).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

        let result = unsafe {
            let service_options = rcl_service_get_default_options();
            rcl_service_init(
                &mut service_handle,
                self.node_handle.as_mut(),
                service_ts,
                service_name_c_string.as_ptr(),
                &service_options,
            )
        };
        if result == RCL_RET_OK as i32 {
            Ok(service_handle)
        } else {
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn create_service<T: 'static>(
        &mut self,
        service_name: &str,
        callback: Box<dyn FnMut(T::Request) -> T::Response>,
    ) -> Result<&rcl_service_t>
    where
        T: WrappedServiceTypeSupport,
    {
        let service_handle = self.create_service_helper(service_name, T::get_ts())?;
        let ws = WrappedService::<T> {
            rcl_handle: service_handle,
            rcl_request: rmw_request_id_t {
                writer_guid: [0; 16usize],
                sequence_number: 0,
            },
            rcl_request_msg: WrappedNativeMsg::<T::Request>::new(),
            callback,
        };

        self.services.push(Box::new(ws));
        Ok(self.services.last().unwrap().handle()) // hmm...
    }

    pub fn create_client_helper(
        &mut self,
        service_name: &str,
        service_ts: *const rosidl_service_type_support_t,
    ) -> Result<rcl_client_t> {
        let mut client_handle = unsafe { rcl_get_zero_initialized_client() };
        let service_name_c_string =
            CString::new(service_name).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

        let result = unsafe {
            let client_options = rcl_client_get_default_options();
            rcl_client_init(
                &mut client_handle,
                self.node_handle.as_mut(),
                service_ts,
                service_name_c_string.as_ptr(),
                &client_options,
            )
        };
        if result == RCL_RET_OK as i32 {
            Ok(client_handle)
        } else {
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn create_client<T: 'static>(&mut self, service_name: &str) -> Result<Client<T>>
    where
        T: WrappedServiceTypeSupport,
    {
        let client_handle = self.create_client_helper(service_name, T::get_ts())?;
        let ws = WrappedClient::<T> {
            rcl_handle: client_handle,
            response_channels: Vec::new(),
        };

        let arc = Arc::new(Mutex::new(ws));
        let client_ = Arc::downgrade(&arc);
        self.clients.push((client_handle, arc));
        let c = Client { client_ };
        Ok(c)
    }

    pub fn service_available<T: 'static + WrappedServiceTypeSupport>(
        &self,
        client: &Client<T>,
    ) -> Result<bool> {
        let client = client
            .client_
            .upgrade()
            .ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let client = client.lock().unwrap();
        let mut avail = false;
        let result = unsafe {
            rcl_service_server_is_available(self.node_handle.as_ref(), client.handle(), &mut avail)
        };

        if result == RCL_RET_OK as i32 {
            Ok(avail)
        } else {
            eprintln!("coult not send request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn create_action_client_helper(
        &mut self,
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
                self.node_handle.as_mut(),
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

    pub fn create_action_client<T: 'static>(&mut self, action_name: &str) -> Result<ActionClient<T>>
    where
        T: WrappedActionTypeSupport,
    {
        let client_handle = self.create_action_client_helper(action_name, T::get_ts())?;
        let wa = WrappedActionClient::<T> {
            rcl_handle: client_handle,
            goal_response_channels: Vec::new(),
            cancel_response_channels: Vec::new(),
            feedback_senders: Vec::new(),
            result_senders: Vec::new(),
            result_requests: Vec::new(),
            goal_status: HashMap::new(),
        };

        let arc = Arc::new(Mutex::new(wa));
        let client = Arc::downgrade(&arc);
        self.action_clients.push((client_handle, arc));
        let c = ActionClient { client };
        Ok(c)
    }

    pub fn action_server_available<T: 'static + WrappedActionTypeSupport>(
        &self,
        client: &ActionClient<T>,
    ) -> Result<bool> {
        let client = client
            .client
            .upgrade()
            .ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let client = client.lock().unwrap();
        let mut avail = false;
        let result = unsafe {
            rcl_action_server_is_available(self.node_handle.as_ref(), client.handle(), &mut avail)
        };

        if result == RCL_RET_OK as i32 {
            Ok(avail)
        } else {
            eprintln!("coult not send request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn create_action_server_helper(
        &mut self,
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
                self.node_handle.as_mut(),
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

    pub fn create_action_server<T: 'static>(
        &mut self,
        action_name: &str,
        accept_goal_cb: Box<dyn FnMut(&uuid::Uuid, &T::Goal) -> bool>,
        accept_cancel_cb: Box<dyn FnMut(&ServerGoal<T>) -> bool>,
        goal_cb: Box<dyn FnMut(ServerGoal<T>)>,
    ) -> Result<ActionServer<T>>
    where
        T: WrappedActionTypeSupport,
    {
        // for now automatically create a ros clock...
        let mut clock_handle = MaybeUninit::<rcl_clock_t>::uninit();
        let ret = unsafe {
            rcl_ros_clock_init(
                clock_handle.as_mut_ptr(),
                &mut rcutils_get_default_allocator(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not create steady clock: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }
        let mut clock_handle = Box::new(unsafe { clock_handle.assume_init() });

        let server_handle =
            self.create_action_server_helper(action_name, clock_handle.as_mut(), T::get_ts())?;
        let wa = WrappedActionServer::<T> {
            rcl_handle: server_handle,
            clock_handle,
            accept_goal_cb,
            accept_cancel_cb,
            goal_cb,
            goals: HashMap::new(),
            result_msgs: HashMap::new(),
            result_requests: HashMap::new(),
        };

        let arc = Arc::new(Mutex::new(wa));
        let server = Arc::downgrade(&arc);
        self.action_servers.push((server_handle, arc));
        let c = ActionServer { server };
        Ok(c)
    }

    pub fn create_publisher_helper(
        &mut self,
        topic: &str,
        typesupport: *const rosidl_message_type_support_t,
    ) -> Result<rcl_publisher_t> {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let topic_c_string = CString::new(topic).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

        let result = unsafe {
            let mut publisher_options = rcl_publisher_get_default_options();
            publisher_options.qos = rmw_qos_profile_t::default();
            rcl_publisher_init(
                &mut publisher_handle,
                self.node_handle.as_mut(),
                typesupport,
                topic_c_string.as_ptr(),
                &publisher_options,
            )
        };
        if result == RCL_RET_OK as i32 {
            Ok(publisher_handle)
        } else {
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn create_publisher<T>(&mut self, topic: &str) -> Result<Publisher<T>>
    where
        T: WrappedTypesupport,
    {
        let publisher_handle = self.create_publisher_helper(topic, T::get_ts())?;
        self.pubs.push(Arc::new(publisher_handle));
        let p = Publisher {
            handle: Arc::downgrade(self.pubs.last().unwrap()),
            type_: PhantomData,
        };
        Ok(p)
    }

    pub fn create_publisher_untyped(
        &mut self,
        topic: &str,
        topic_type: &str,
    ) -> Result<PublisherUntyped> {
        let dummy = WrappedNativeMsgUntyped::new_from(topic_type)?; // TODO, get ts without allocating msg
        let publisher_handle = self.create_publisher_helper(topic, dummy.ts)?;

        self.pubs.push(Arc::new(publisher_handle));
        let p = PublisherUntyped {
            handle: Arc::downgrade(self.pubs.last().unwrap()),
            type_: topic_type.to_owned(),
        };
        Ok(p)
    }

    fn action_client_get_num_waits(
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

    fn action_server_get_num_waits(
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

    pub fn spin_once(&mut self, timeout: Duration) {
        let timeout = timeout.as_nanos() as i64;
        let mut ws = unsafe { rcl_get_zero_initialized_wait_set() };

        // #[doc = "* This function is thread-safe for unique wait sets with unique contents."]
        // #[doc = "* This function cannot operate on the same wait set in multiple threads, and"]
        // #[doc = "* the wait sets may not share content."]
        // #[doc = "* For example, calling rcl_wait() in two threads on two different wait sets"]
        // #[doc = "* that both contain a single, shared guard condition is undefined behavior."]

        // count action client wait set needs
        let mut total_action_subs = 0;
        let mut total_action_clients = 0;
        for (ach, _) in &self.action_clients {
            let mut num_subs = 0;
            let mut num_gc = 0;
            let mut num_timers = 0;
            let mut num_clients = 0;
            let mut num_services = 0;

            Self::action_client_get_num_waits(
                ach,
                &mut num_subs,
                &mut num_gc,
                &mut num_timers,
                &mut num_clients,
                &mut num_services,
            )
            .expect("could not get action client wait sets");
            // sanity check
            assert_eq!(num_subs, 2);
            assert_eq!(num_clients, 3);
            assert_eq!(num_gc, 0);
            assert_eq!(num_timers, 0);
            assert_eq!(num_services, 0);

            total_action_subs += num_subs;
            total_action_clients += num_clients;
        }

        // count action server wait set needs
        let mut total_action_timers = 0;
        let mut total_action_services = 0;
        for (ash, _) in &self.action_servers {
            let mut num_subs = 0;
            let mut num_gc = 0;
            let mut num_timers = 0;
            let mut num_clients = 0;
            let mut num_services = 0;

            Self::action_server_get_num_waits(
                ash,
                &mut num_subs,
                &mut num_gc,
                &mut num_timers,
                &mut num_clients,
                &mut num_services,
            )
            .expect("could not get action client wait sets");
            // sanity check
            assert_eq!(num_subs, 0);
            assert_eq!(num_clients, 0);
            assert_eq!(num_gc, 0);
            assert_eq!(num_timers, 1);
            assert_eq!(num_services, 3);

            total_action_timers += num_timers;
            total_action_services += num_services;
        }

        {
            let mut ctx = self.context.context_handle.lock().unwrap();

            unsafe {
                rcl_wait_set_init(
                    &mut ws,
                    self.subs.len() + total_action_subs,
                    0,
                    self.timers.len() + total_action_timers,
                    self.clients.len() + total_action_clients,
                    self.services.len() + total_action_services,
                    0,
                    ctx.as_mut(),
                    rcutils_get_default_allocator(),
                );
            }
        }
        unsafe {
            rcl_wait_set_clear(&mut ws);
        }

        for s in self.subs.iter() {
            unsafe {
                rcl_wait_set_add_subscription(&mut ws, s.handle(), std::ptr::null_mut());
            }
        }

        for s in self.timers.iter() {
            unsafe {
                rcl_wait_set_add_timer(&mut ws, &s.timer_handle, std::ptr::null_mut());
            }
        }

        for (handle, _) in self.clients.iter() {
            unsafe {
                rcl_wait_set_add_client(&mut ws, &*handle, std::ptr::null_mut());
            }
        }

        for s in self.services.iter() {
            unsafe {
                rcl_wait_set_add_service(&mut ws, s.handle(), std::ptr::null_mut());
            }
        }

        // code below assumes that actions are added last... perhaps a
        // bad assumption.  e.g. we add subscriptions and timers of
        // the node before ones created automatically by actions. we
        // then assume that we can count on the waitables created by
        // the actions are added at the end of the wait set arrays
        for (ac, _) in self.action_clients.iter() {
            unsafe {
                rcl_action_wait_set_add_action_client(
                    &mut ws,
                    ac,
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                );
            }
        }
        for (ash, _) in self.action_servers.iter() {
            unsafe {
                rcl_action_wait_set_add_action_server(&mut ws, ash, std::ptr::null_mut());
            }
        }

        let ret = unsafe { rcl_wait(&mut ws, timeout) };

        if ret == RCL_RET_TIMEOUT as i32 {
            unsafe {
                rcl_wait_set_fini(&mut ws);
            }
            return;
        }

        let ws_subs = unsafe { std::slice::from_raw_parts(ws.subscriptions, self.subs.len()) };
        for (s, ws_s) in self.subs.iter_mut().zip(ws_subs) {
            if ws_s != &std::ptr::null() {
                s.handle_incoming();
            }
        }

        let ws_timers = unsafe { std::slice::from_raw_parts(ws.timers, self.timers.len()) };
        let mut timers_to_remove = vec![];
        for (s, ws_s) in self.timers.iter_mut().zip(ws_timers) {
            if ws_s != &std::ptr::null() {
                let mut is_ready = false;
                let ret = unsafe { rcl_timer_is_ready(&s.timer_handle, &mut is_ready) };
                if ret == RCL_RET_OK as i32 {
                    if is_ready {
                        let mut nanos = 0i64;
                        // todo: error handling
                        let ret = unsafe {
                            rcl_timer_get_time_since_last_call(&s.timer_handle, &mut nanos)
                        };
                        if ret == RCL_RET_OK as i32 {
                            let ret = unsafe { rcl_timer_call(&mut s.timer_handle) };
                            if ret == RCL_RET_OK as i32 {
                                match s.sender.try_send(Duration::from_nanos(nanos as u64)) {
                                    Err(e) => {
                                        if e.is_full() {
                                            println!("Warning: timer tick not handled in time - no wakeup will occur");
                                        }
                                        if e.is_disconnected() {
                                            // client dropped the timer handle, let's drop our timer as well.
                                            timers_to_remove.push(s.timer_handle);
                                        }
                                    }
                                    _ => {} // ok
                                }
                            }
                        }
                    }
                }
            }
        }
        // drop timers scheduled for deletion
        self.timers
            .retain(|t| !timers_to_remove.contains(&t.timer_handle));

        let ws_clients = unsafe { std::slice::from_raw_parts(ws.clients, self.clients.len()) };
        for ((_, s), ws_s) in self.clients.iter_mut().zip(ws_clients) {
            if ws_s != &std::ptr::null() {
                let mut s = s.lock().unwrap();
                s.handle_response();
            }
        }

        let ws_services = unsafe { std::slice::from_raw_parts(ws.services, self.services.len()) };
        for (s, ws_s) in self.services.iter_mut().zip(ws_services) {
            if ws_s != &std::ptr::null() {
                let ret = unsafe {
                    rcl_take_request(s.handle(), s.rcl_request_id(), s.rcl_request_msg())
                };
                if ret == RCL_RET_OK as i32 {
                    s.run_cb();
                }
            }
        }

        for (ac, s) in &self.action_clients {
            let mut is_feedback_ready = false;
            let mut is_status_ready = false;
            let mut is_goal_response_ready = false;
            let mut is_cancel_response_ready = false;
            let mut is_result_response_ready = false;

            let ret = unsafe {
                rcl_action_client_wait_set_get_entities_ready(
                    &ws,
                    ac,
                    &mut is_feedback_ready,
                    &mut is_status_ready,
                    &mut is_goal_response_ready,
                    &mut is_cancel_response_ready,
                    &mut is_result_response_ready,
                )
            };

            if ret != RCL_RET_OK as i32 {
                continue;
            }

            if is_feedback_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_feedback_msg();
            }

            if is_status_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_status_msg();
            }

            if is_goal_response_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_goal_response();
            }

            if is_cancel_response_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_cancel_response();
            }

            if is_result_response_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_result_response();
            }
        }

        for (ash, s) in &self.action_servers {
            let mut is_goal_request_ready = false;
            let mut is_cancel_request_ready = false;
            let mut is_result_request_ready = false;
            let mut is_goal_expired = false;

            let ret = unsafe {
                rcl_action_server_wait_set_get_entities_ready(
                    &ws,
                    ash,
                    &mut is_goal_request_ready,
                    &mut is_cancel_request_ready,
                    &mut is_result_request_ready,
                    &mut is_goal_expired,
                )
            };

            if ret != RCL_RET_OK as i32 {
                continue;
            }

            if is_goal_request_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_goal_request(s.clone());
            }

            if is_cancel_request_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_cancel_request();
            }

            if is_result_request_ready {
                let mut acs = s.lock().unwrap();
                acs.handle_result_request();
            }

            if is_goal_expired {
                let mut acs = s.lock().unwrap();
                acs.handle_goal_expired();
            }
        }

        unsafe {
            rcl_wait_set_fini(&mut ws);
        }
    }

    pub fn get_topic_names_and_types(&self) -> Result<HashMap<String, Vec<String>>> {
        let mut tnat = unsafe { rmw_get_zero_initialized_names_and_types() };
        let ret = unsafe {
            rcl_get_topic_names_and_types(
                self.node_handle.as_ref(),
                &mut rcutils_get_default_allocator(),
                false,
                &mut tnat,
            )
        };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not get topic names and types {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        let names = unsafe { std::slice::from_raw_parts(tnat.names.data, tnat.names.size) };
        let types = unsafe { std::slice::from_raw_parts(tnat.types, tnat.names.size) };

        let mut res = HashMap::new();
        for (n, t) in names.iter().zip(types) {
            let topic_name = unsafe { CStr::from_ptr(*n).to_str().unwrap().to_owned() };
            let topic_types = unsafe { std::slice::from_raw_parts(t, t.size) };
            let topic_types: Vec<String> = unsafe {
                topic_types
                    .iter()
                    .map(|t| CStr::from_ptr(*((*t).data)).to_str().unwrap().to_owned())
                    .collect()
            };
            res.insert(topic_name, topic_types);
        }
        unsafe {
            rmw_names_and_types_fini(&mut tnat);
        } // TODO: check return value
        Ok(res)
    }

    pub fn create_wall_timer(&mut self, period: Duration) -> Result<Timer> {
        let mut clock_handle = MaybeUninit::<rcl_clock_t>::uninit();

        let ret = unsafe {
            rcl_steady_clock_init(
                clock_handle.as_mut_ptr(),
                &mut rcutils_get_default_allocator(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not create steady clock: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        let mut clock_handle = Box::new(unsafe { clock_handle.assume_init() });
        let mut timer_handle = unsafe { rcl_get_zero_initialized_timer() };

        {
            let mut ctx = self.context.context_handle.lock().unwrap();
            let ret = unsafe {
                rcl_timer_init(
                    &mut timer_handle,
                    clock_handle.as_mut(),
                    ctx.as_mut(),
                    period.as_nanos() as i64,
                    None,
                    rcutils_get_default_allocator(),
                )
            };

            if ret != RCL_RET_OK as i32 {
                eprintln!("could not create timer: {}", ret);
                return Err(Error::from_rcl_error(ret));
            }
        }

        let (tx, rx) = mpsc::channel::<Duration>(1);

        let timer = Timer_ {
            timer_handle,
            clock_handle,
            sender: tx,
        };
        self.timers.push(timer);

        let out_timer = Timer { receiver: rx };

        Ok(out_timer)
    }

    pub fn logger<'a>(&'a self) -> &'a str {
        let ptr = unsafe { rcl_node_get_logger_name(self.node_handle.as_ref()) };
        if ptr == std::ptr::null() {
            return "";
        }
        let s = unsafe { CStr::from_ptr(ptr) };
        s.to_str().unwrap_or("")
    }
}

pub struct Timer_ {
    timer_handle: rcl_timer_t,
    clock_handle: Box<rcl_clock_t>,
    sender: mpsc::Sender<Duration>,
}

pub struct Timer {
    receiver: mpsc::Receiver<Duration>,
}

impl Timer {
    pub async fn tick(&mut self) -> Result<Duration> {
        let next = self.receiver.next().await;
        if let Some(elapsed) = next {
            Ok(elapsed)
        } else {
            Err(Error::RCL_RET_TIMER_INVALID)
        }
    }
}

// Since publishers are temporarily upgraded to owners during the
// actual publish but are not the ones that handle cleanup, we simply
// wait until there are no other owners in the cleanup procedure. The
// next time a publisher wants to publish they will fail because the
// value in the Arc has been dropped. Hacky but works.
fn wait_until_unwrapped<T>(mut a: Arc<T>) -> T
where
    T: std::fmt::Debug,
{
    loop {
        match Arc::try_unwrap(a) {
            Ok(b) => return b,
            Err(t) => a = t,
        }
    }
}

impl Drop for Node {
    fn drop(&mut self) {
        // fini functions are not thread safe so lock the context.
        let _ctx_handle = self.context.context_handle.lock().unwrap();

        for s in &mut self.subs {
            s.destroy(&mut self.node_handle);
        }
        for s in &mut self.services {
            s.destroy(&mut self.node_handle);
        }
        for t in &mut self.timers {
            // TODO: check return values
            let _ret = unsafe { rcl_timer_fini(&mut t.timer_handle) };
            // TODO: allow other types of clocks...
            let _ret = unsafe { rcl_steady_clock_fini(t.clock_handle.as_mut()) };
        }
        for (_, c) in &mut self.action_clients {
            c.lock().unwrap().destroy(&mut self.node_handle);
        }
        for (_, s) in &mut self.action_servers {
            s.lock().unwrap().destroy(&mut self.node_handle);
        }
        while let Some(p) = self.pubs.pop() {
            let mut p = wait_until_unwrapped(p);
            let _ret = unsafe { rcl_publisher_fini(&mut p as *mut _, self.node_handle.as_mut()) };
            // TODO: check ret
        }
        unsafe {
            rcl_node_fini(self.node_handle.as_mut());
        }
    }
}

impl<T: 'static> Publisher<T>
where
    T: WrappedTypesupport,
{
    pub fn publish(&self, msg: &T) -> Result<()>
    where
        T: WrappedTypesupport,
    {
        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;
        let native_msg: WrappedNativeMsg<T> = WrappedNativeMsg::<T>::from(msg);
        let result = unsafe {
            rcl_publish(
                publisher.as_ref(),
                native_msg.void_ptr(),
                std::ptr::null_mut(),
            )
        };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            eprintln!("coult not publish {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn publish_native(&self, msg: &WrappedNativeMsg<T>) -> Result<()>
    where
        T: WrappedTypesupport,
    {
        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

        let result =
            unsafe { rcl_publish(publisher.as_ref(), msg.void_ptr(), std::ptr::null_mut()) };
        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            eprintln!("could not publish native {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}

impl<T: 'static> Client<T>
where
    T: WrappedServiceTypeSupport,
{
    pub fn request(&self, msg: &T::Request) -> Result<impl Future<Output = Result<T::Response>>>
    where
        T: WrappedServiceTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let client = self
            .client_
            .upgrade()
            .ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();
        let native_msg: WrappedNativeMsg<T::Request> = WrappedNativeMsg::<T::Request>::from(msg);
        let mut seq_no = 0i64;
        let result =
            unsafe { rcl_send_request(&client.rcl_handle, native_msg.void_ptr(), &mut seq_no) };

        let (sender, receiver) = oneshot::channel::<T::Response>();

        if result == RCL_RET_OK as i32 {
            client.response_channels.push((seq_no, sender));
            // instead of "canceled" we return invalid client.
            Ok(receiver.map_err(|_| Error::RCL_RET_CLIENT_INVALID))
        } else {
            eprintln!("coult not send request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}

impl<T: 'static> ClientGoal<T>
where
    T: WrappedActionTypeSupport,
{
    pub fn get_status(&self) -> Result<GoalStatus> {
        let client = self
            .client
            .upgrade()
            .ok_or(Error::RCL_RET_ACTION_CLIENT_INVALID)?;
        let client = client.lock().unwrap();

        Ok(client.get_goal_status(&self.uuid))
    }

    pub fn get_result(&mut self) -> Result<impl Future<Output = Result<T::Result>>> {
        if let Some(result_channel) = self.result.lock().unwrap().take() {
            // upgrade to actual ref. if still alive
            let client = self
                .client
                .upgrade()
                .ok_or(Error::RCL_RET_ACTION_CLIENT_INVALID)?;
            let mut client = client.lock().unwrap();

            client.send_result_request(self.uuid);

            Ok(result_channel.map_err(|_| Error::RCL_RET_ACTION_CLIENT_INVALID))
        } else {
            // todo: error codes...
            println!("already asked for the result!");
            Err(Error::RCL_RET_ACTION_CLIENT_INVALID)
        }
    }

    pub fn get_feedback(&self) -> Result<impl Stream<Item = T::Feedback> + Unpin> {
        if let Some(feedback_channel) = self.feedback.lock().unwrap().take() {
            Ok(feedback_channel)
        } else {
            // todo: error codes...
            println!("someone else owns the feedback consumer stream");
            Err(Error::RCL_RET_ACTION_CLIENT_INVALID)
        }
    }

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
    pub fn send_goal_request(
        &self,
        goal: T::Goal,
    ) -> Result<impl Future<Output = Result<ClientGoal<T>>>>
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
        let (goal_req_sender, goal_req_receiver) = oneshot::channel::<
            <<T as WrappedActionTypeSupport>::SendGoal as WrappedServiceTypeSupport>::Response,
        >();
        let (feedback_sender, feedback_receiver) = mpsc::channel::<T::Feedback>(1);
        client.feedback_senders.push((uuid, feedback_sender));
        let (result_sender, result_receiver) = oneshot::channel::<T::Result>();
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
                    Ok(resp) => {
                        let (accepted, _stamp) = T::destructure_goal_response_msg(resp);
                        if accepted {
                            Ok(ClientGoal {
                                client: fut_client,
                                uuid,
                                feedback: Arc::new(Mutex::new(Some(feedback_receiver))),
                                result: Arc::new(Mutex::new(Some(result_receiver))),
                            })
                        } else {
                            println!("goal rejected");
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

impl PublisherUntyped {
    pub fn publish(&self, msg: serde_json::Value) -> Result<()> {
        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

        let mut native_msg = WrappedNativeMsgUntyped::new_from(&self.type_)?;
        native_msg.from_json(msg)?;

        let result = unsafe {
            rcl_publish(
                publisher.as_ref(),
                native_msg.void_ptr(),
                std::ptr::null_mut(),
            )
        };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            eprintln!("coult not publish {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}

#[derive(Debug)]
pub enum ClockType {
    RosTime,
    SystemTime,
    SteadyTime,
}

unsafe impl Send for Clock {}

pub struct Clock {
    clock_handle: Box<rcl_clock_t>,
}

impl Clock {
    fn clock_type_to_rcl(ct: &ClockType) -> rcl_clock_type_t {
        match ct {
            ClockType::RosTime => rcl_clock_type_t::RCL_ROS_TIME,
            ClockType::SystemTime => rcl_clock_type_t::RCL_SYSTEM_TIME,
            ClockType::SteadyTime => rcl_clock_type_t::RCL_STEADY_TIME,
        }
    }

    pub fn create(ct: ClockType) -> Result<Self> {
        let mut clock_handle = MaybeUninit::<rcl_clock_t>::uninit();

        let rcl_ct = Clock::clock_type_to_rcl(&ct);
        let ret = unsafe {
            rcl_clock_init(
                rcl_ct,
                clock_handle.as_mut_ptr(),
                &mut rcutils_get_default_allocator(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not create {:?} clock: {}", ct, ret);
            return Err(Error::from_rcl_error(ret));
        }

        let clock_handle = Box::new(unsafe { clock_handle.assume_init() });
        Ok(Clock { clock_handle })
    }

    pub fn get_now(&mut self) -> Result<Duration> {
        let valid = unsafe { rcl_clock_valid(&mut *self.clock_handle) };
        if !valid {
            return Err(Error::from_rcl_error(RCL_RET_INVALID_ARGUMENT as i32));
        }
        let mut tp: rcutils_time_point_value_t = 0;
        let ret = unsafe { rcl_clock_get_now(&mut *self.clock_handle, &mut tp) };

        if ret != RCL_RET_OK as i32 {
            eprintln!("could not create steady clock: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        let dur = Duration::from_nanos(tp as u64);

        Ok(dur)
    }

    pub fn to_builtin_time(d: &Duration) -> builtin_interfaces::msg::Time {
        let sec = d.as_secs() as i32;
        let nanosec = d.subsec_nanos();
        builtin_interfaces::msg::Time { sec, nanosec }
    }
}

impl Drop for Clock {
    fn drop(&mut self) {
        unsafe {
            rcl_clock_fini(&mut *self.clock_handle);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_context_drop() -> () {
        {
            let ctx = Context::create().unwrap();
            assert!(ctx.is_valid());
        }
        {
            let ctx = Context::create().unwrap();
            assert!(ctx.is_valid());
        }
    }

    #[test]
    fn test_ros_str() -> () {
        let hej = "hej hopp";
        let mut msg = WrappedNativeMsg::<std_msgs::msg::String>::new();
        msg.data.assign(hej);
        assert_eq!(msg.data.to_str(), hej);
    }

    #[test]
    fn test_copy_fields() -> () {
        let msg_orig = std_msgs::msg::String { data: "hej".into() };
        let rosmsg = WrappedNativeMsg::<std_msgs::msg::String>::from(&msg_orig);
        let msg2 = std_msgs::msg::String::from_native(&rosmsg);
        assert_eq!(msg_orig, msg2);
    }

    #[test]
    fn test_introspection_string() -> () {
        unsafe {
            use std::ffi::CStr;

            let x = rosidl_typesupport_introspection_c__get_message_type_support_handle__std_msgs__msg__String();
            let members = (*x).data as *const rosidl_typesupport_introspection_c__MessageMembers;
            println!("{:#?}", *members);

            assert_eq!((*members).member_count_, 1);

            let s = CStr::from_ptr((*members).message_namespace_)
                .to_str()
                .unwrap();
            assert_eq!(s, "std_msgs__msg");

            let s = CStr::from_ptr((*members).message_name_).to_str().unwrap();
            assert_eq!(s, "String");

            let member = (*members).members_;
            println!("member: {:#?}", *member);
            let field_name = CStr::from_ptr((*member).name_).to_str().unwrap();
            let type_id = (*member).type_id_;
            let is_array = (*member).is_array_;
            assert_eq!(field_name, "data");
            assert_eq!(
                type_id,
                rosidl_typesupport_introspection_c__ROS_TYPE_STRING as u8
            );
            assert_eq!(is_array, false);
        }
    }

    #[test]
    #[should_panic] // we are testing that we cannot have to many elements in a fixed sized field
    fn test_fixedsizearray() -> () {
        unsafe {
            let x = rosidl_typesupport_introspection_c__get_message_type_support_handle__geometry_msgs__msg__AccelWithCovariance();
            let members = (*x).data as *const rosidl_typesupport_introspection_c__MessageMembers;
            println!("{:#?}", *members);

            let memberslice =
                std::slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);
            for member in memberslice {
                println!("member: {:#?}", *member);
            }

            let msg_native = WrappedNativeMsg::<geometry_msgs::msg::AccelWithCovariance>::new();
            let mut msg = geometry_msgs::msg::AccelWithCovariance::from_native(&msg_native);
            println!("{:#?}", msg);
            msg.covariance[0] = 10.0;
            msg.covariance[10] = 10.0;
            msg.covariance[35] = 99.0;
            msg.covariance.push(4444.0);
            let msg_native2 =
                WrappedNativeMsg::<geometry_msgs::msg::AccelWithCovariance>::from(&msg);
            let msg2 = geometry_msgs::msg::AccelWithCovariance::from_native(&msg_native2);
            println!("{:#?}", msg2);
        }
    }

    #[test]
    #[should_panic] // we are testing that we cannot have to many elements in a fixed sized field
    fn test_capped_sequence() -> () {
        // float64[<=3] dimensions in the .msg translates to a float64 sequence AND an array size field. handle it...
        unsafe {
            let x = rosidl_typesupport_introspection_c__get_message_type_support_handle__shape_msgs__msg__SolidPrimitive();
            let members = (*x).data as *const rosidl_typesupport_introspection_c__MessageMembers;
            println!("{:#?}", *members);

            let memberslice =
                std::slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);
            for member in memberslice {
                println!("member: {:#?}", *member);
            }

            let msg_native = WrappedNativeMsg::<shape_msgs::msg::SolidPrimitive>::new();
            let mut msg = shape_msgs::msg::SolidPrimitive::from_native(&msg_native);
            println!("{:#?}", msg);
            msg.dimensions.push(1.0);
            msg.dimensions.push(1.0);
            msg.dimensions.push(1.0);
            msg.dimensions.push(1.0); // only three elements allowed
            let _msg_native2 = WrappedNativeMsg::<shape_msgs::msg::SolidPrimitive>::from(&msg);
        }
    }

    #[test]
    fn test_generation_string_use() -> () {
        let msg = std_msgs::msg::String { data: "hej".into() };
        let msg2 = msg.clone();
        let msg_native = WrappedNativeMsg::<std_msgs::msg::String>::from(&msg2);
        let msg2 = std_msgs::msg::String::from_native(&msg_native);
        assert_eq!(msg, msg2)
    }

    #[test]
    fn test_generation_bool_use() -> () {
        let msg = std_msgs::msg::Bool { data: true };
        let msg_native = WrappedNativeMsg::<std_msgs::msg::Bool>::from(&msg);
        let msg2 = std_msgs::msg::Bool::from_native(&msg_native);
        assert_eq!(msg, msg2);
    }

    #[test]
    fn test_float_sequence() -> () {
        use trajectory_msgs::msg::*;
        let native = WrappedNativeMsg::<JointTrajectoryPoint>::new();
        let mut msg = JointTrajectoryPoint::from_native(&native);
        msg.positions.push(39.0);
        msg.positions.push(34.0);
        let new_native = WrappedNativeMsg::<JointTrajectoryPoint>::from(&msg);
        let new_msg = JointTrajectoryPoint::from_native(&new_native);
        println!("{:#?}", new_msg);
        assert_eq!(msg, new_msg);
    }

    #[test]
    fn test_deault() -> () {
        use trajectory_msgs::msg::*;
        let mut msg: JointTrajectoryPoint = Default::default();
        msg.positions.push(39.0);
        msg.positions.push(34.0);
        let mut new_native = WrappedNativeMsg::<JointTrajectoryPoint>::from(&msg);
        unsafe { *((*new_native).positions.data) = 88.9 };
        let new_msg = JointTrajectoryPoint::from_native(&new_native);
        println!("{:#?}", new_msg);
        assert_ne!(msg, new_msg);
    }

    #[test]
    fn test_untyped_json() -> () {
        let mut msg = trajectory_msgs::msg::JointTrajectoryPoint::default();
        msg.positions.push(39.0);
        msg.positions.push(34.0);
        let json = serde_json::to_value(msg.clone()).unwrap();

        let mut native =
            WrappedNativeMsgUntyped::new_from("trajectory_msgs/msg/JointTrajectoryPoint").unwrap();
        native.from_json(json.clone()).unwrap();
        let json2 = native.to_json().unwrap();
        assert_eq!(json, json2);

        let msg2: trajectory_msgs::msg::JointTrajectoryPoint =
            serde_json::from_value(json2).unwrap();
        assert_eq!(msg, msg2);
    }

    #[cfg(r2r__test_msgs__msg__Arrays)]
    #[test]
    fn test_test_msgs_array() -> () {
        let mut msg = test_msgs::msg::Arrays::default();
        println!("msg: {:?}", msg.string_values);
        msg.string_values = vec!["hej".to_string(), "hopp".to_string(), "stropp".to_string()];

        let msg_native = WrappedNativeMsg::<test_msgs::msg::Arrays>::from(&msg);
        let msg2 = test_msgs::msg::Arrays::from_native(&msg_native);

        assert_eq!(msg, msg2);
    }

    #[cfg(r2r__test_msgs__msg__Arrays)]
    #[test]
    #[should_panic]
    fn test_test_msgs_array_too_few_elems() -> () {
        let mut msg = test_msgs::msg::Arrays::default();
        println!("msg: {:?}", msg.string_values);
        msg.string_values = vec!["hej".to_string(), "hopp".to_string()];
        let _msg_native = WrappedNativeMsg::<test_msgs::msg::Arrays>::from(&msg);
    }

    #[cfg(r2r__test_msgs__msg__WStrings)]
    #[test]
    fn test_test_msgs_wstring() -> () {
        let mut msg = test_msgs::msg::WStrings::default();
        let rust_str = "ハローワールド";
        msg.wstring_value = rust_str.to_string();
        let native = WrappedNativeMsg::<test_msgs::msg::WStrings>::from(&msg);
        println!("msg: {:?}", msg);
        let msg2 = test_msgs::msg::WStrings::from_native(&native);
        assert_eq!(msg.wstring_value, msg2.wstring_value);
    }

    #[cfg(r2r__example_interfaces__srv__AddTwoInts)]
    #[test]
    fn test_service_msgs() {
        use example_interfaces::srv::AddTwoInts;
        let mut req = AddTwoInts::Request::default();
        req.a = 5;
        let rn = WrappedNativeMsg::<_>::from(&req);
        let req2 = AddTwoInts::Request::from_native(&rn);
        println!("req2 {:?}", req2);
        assert_eq!(req, req2);

        let mut resp = AddTwoInts::Response::default();
        resp.sum = 5;
        let rn = WrappedNativeMsg::<_>::from(&resp);
        let resp2 = AddTwoInts::Response::from_native(&rn);
        println!("resp {:?}", resp2);
        assert_eq!(resp, resp2);
    }

    #[cfg(r2r__example_interfaces__action__Fibonacci)]
    #[test]
    fn test_action_msgs() {
        use example_interfaces::action::Fibonacci;
        let mut goal = Fibonacci::Goal::default();
        goal.order = 5;
        let gn = WrappedNativeMsg::<_>::from(&goal);
        let goal2 = Fibonacci::Goal::from_native(&gn);
        println!("goal2 {:?}", goal2);
        assert_eq!(goal, goal2);

        let mut res = Fibonacci::Result::default();
        res.sequence = vec![1, 2, 3];
        let rn = WrappedNativeMsg::<_>::from(&res);
        let res2 = Fibonacci::Result::from_native(&rn);
        println!("res2 {:?}", res2);
        assert_eq!(res, res2);

        let mut fb = Fibonacci::Feedback::default();
        fb.sequence = vec![4, 3, 6];
        let fbn = WrappedNativeMsg::<_>::from(&fb);
        let fb2 = Fibonacci::Feedback::from_native(&fbn);
        println!("feedback2 {:?}", fb2);
        assert_eq!(fb, fb2);

        let fb = WrappedNativeMsg::<Fibonacci::Feedback>::new();
        let fb1 = Fibonacci::Feedback::default();
        let fb2 = Fibonacci::Feedback::from_native(&fb);
        assert_eq!(fb1, fb2);
    }
}
