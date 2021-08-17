use crate::error::*;
use msg_gen::*;
use rcl::{
    rosidl_action_type_support_t, rosidl_message_type_support_t, rosidl_service_type_support_t,
};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use std::ops::{Deref, DerefMut};

pub mod generated_msgs {
    use super::*;
    include!(concat!(env!("OUT_DIR"), "/_r2r_generated_msgs.rs"));
    include!(concat!(
        env!("OUT_DIR"),
        "/_r2r_generated_untyped_helper.rs"
    ));
    include!(concat!(
        env!("OUT_DIR"),
        "/_r2r_generated_service_helper.rs"
    ));
}

use generated_msgs::{builtin_interfaces, unique_identifier_msgs};

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
    pub ts: &'static rosidl_message_type_support_t,
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

pub struct UntypedServiceSupport {
    pub ts: &'static rosidl_service_type_support_t,
    pub make_request_msg: fn() -> WrappedNativeMsgUntyped,
    pub make_response_msg: fn() -> WrappedNativeMsgUntyped,
}

impl UntypedServiceSupport {
    fn new<T>() -> Self
    where
        T: WrappedServiceTypeSupport,
    {
        let make_request_msg = || WrappedNativeMsgUntyped::new::<T::Request>();
        let make_response_msg = || WrappedNativeMsgUntyped::new::<T::Response>();

        UntypedServiceSupport {
            ts: T::get_ts(),
            make_request_msg,
            make_response_msg,
        }
    }
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

    pub fn to_json(&self) -> Result<serde_json::Value> {
        let json = (self.msg_to_json)(self.msg);
        json.map_err(|serde_err| Error::SerdeError {
            err: serde_err.to_string(),
        })
    }

    pub fn from_json(&mut self, json: serde_json::Value) -> Result<()> {
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

#[cfg(test)]
mod tests {
    use super::generated_msgs::*;
    use super::*;
    use rcl::*;

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

    #[cfg(r2r__std_srvs__srv__Empty)]
    #[test]
    fn test_empty_msgs() {
        use std_srvs::srv::Empty;
        let req = Empty::Request::default();
        let resp = Empty::Response::default();
        println!("req {:?}", req);
        println!("resp {:?}", resp);
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
