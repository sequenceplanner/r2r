include!("./generated_msgs.rs");

use msg_gen::*;
use rcl::*;

use std::ffi::CString;
use std::ops::{Deref, DerefMut};
use serde::{Serialize, Deserialize};

pub trait WrappedTypesupport {
    type CStruct;

    fn get_ts() -> &'static rosidl_message_type_support_t;
    fn create_msg() -> *mut Self::CStruct;
    fn destroy_msg(msg: *mut Self::CStruct);
    fn from_native(msg: &Self::CStruct) -> Self;
    fn copy_to_native(&self, msg: &mut Self::CStruct);
}

#[derive(Debug)]
pub struct WrappedNativeMsg<T>
where
    T: WrappedTypesupport,
{
    pub msg: *mut T::CStruct,
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

    pub fn void_ptr(&self) -> *const std::os::raw::c_void {
        self.msg as *const _ as *const std::os::raw::c_void
    }

    pub fn void_ptr_mut(&mut self) -> *mut std::os::raw::c_void {
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
    fn run_cb(&mut self) -> ();
    fn rcl_msg(&mut self) -> *mut std::os::raw::c_void;
}

pub struct WrappedSubT<T>
where
    T: WrappedTypesupport,
{
    pub rcl_handle: rcl_subscription_t,
    pub callback: Box<dyn FnMut(T) -> ()>,
    pub rcl_msg: WrappedNativeMsg<T>,
}

pub struct WrappedSubNative<T>
where
    T: WrappedTypesupport,
{
    pub rcl_handle: rcl_subscription_t,
    pub callback: Box<dyn FnMut(&WrappedNativeMsg<T>) -> ()>,
    pub rcl_msg: WrappedNativeMsg<T>,
}

impl<T> Sub for WrappedSubT<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn rcl_msg(&mut self) -> *mut std::os::raw::c_void {
        self.rcl_msg.void_ptr_mut()
    }

    fn run_cb(&mut self) -> () {
        // copy native msg to rust type and run callback
        let msg = T::from_native(&self.rcl_msg);
        (self.callback)(msg);
    }
}

impl<T> Sub for WrappedSubNative<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn rcl_msg(&mut self) -> *mut std::os::raw::c_void {
        self.rcl_msg.void_ptr_mut()
    }

    fn run_cb(&mut self) -> () {
        // *dont't* copy native msg to rust type.
        // e.g. if you for instance have large image data...
        (self.callback)(&self.rcl_msg);
    }
}

pub fn rcl_create_context() -> Result<rcl_context_t, ()> {
    let mut ctx = unsafe { rcl_get_zero_initialized_context() };
    let isok = unsafe {
        let allocator = rcutils_get_default_allocator();
        let mut init_options = rcl_get_zero_initialized_init_options();
        rcl_init_options_init(&mut init_options, allocator);
        rcl_init(0, std::ptr::null(), &init_options, &mut ctx);
        let isok = rcl_context_is_valid(&mut ctx);
        rcl_init_options_fini(&mut init_options as *mut _);
        isok
    };

    if isok {
        Ok(ctx)
    } else {
        Err(())
    }
}

pub fn rcl_create_node(
    ctx: &mut rcl_context_t,
    name: &str,
    namespace: &str,
) -> Result<rcl_node_t, ()> {
    let c_node_name = CString::new(name).unwrap();
    let c_node_ns = CString::new(namespace).unwrap();
    let mut node_handle = unsafe { rcl_get_zero_initialized_node() };
    let nr = unsafe {
        let node_options = rcl_node_get_default_options();
        rcl_node_init(
            &mut node_handle as *mut _,
            c_node_name.as_ptr(),
            c_node_ns.as_ptr(),
            ctx,
            &node_options as *const _,
        )
    };
    if nr == RCL_RET_OK as i32 {
        Ok(node_handle)
    } else {
        eprintln!("{}", nr);
        Err(())
    }
}

pub fn rcl_destroy_node(node: &mut rcl_node_t) {
    unsafe {
        rcl_node_fini(node);
    }
}

pub fn rcl_destroy_ctx(ctx: &mut rcl_context_t) {
    unsafe {
        rcl_shutdown(ctx);
    }
}

pub fn rcl_create_publisher<T>(node: &mut rcl_node_t, topic: &str) -> Result<rcl_publisher_t, ()>
where
    T: WrappedTypesupport,
{
    let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
    let topic_c_string = CString::new(topic).unwrap();

    let result = unsafe {
        let mut publisher_options = rcl_publisher_get_default_options();
        publisher_options.qos = rmw_qos_profile_t::default();
        rcl_publisher_init(
            &mut publisher_handle,
            node,
            T::get_ts(),
            topic_c_string.as_ptr(),
            &publisher_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(publisher_handle)
    } else {
        eprintln!("{}", result);
        Err(())
    }
}

pub fn publish<T>(publisher: &rcl_publisher_t, msg: &T) -> Result<(), ()>
where
    T: WrappedTypesupport,
{
    // copy rust msg to native and publish it
    let native_msg: WrappedNativeMsg<T> = WrappedNativeMsg::<T>::from(msg);
    let result = unsafe { rcl_publish(publisher, native_msg.void_ptr(), std::ptr::null_mut()) };

    if result == RCL_RET_OK as i32 {
        Ok(())
    } else {
        eprintln!("{}", result);
        Err(())
    }
}

pub fn publish_native<T>(publisher: &rcl_publisher_t, msg: &WrappedNativeMsg<T>) -> Result<(), ()>
where
    T: WrappedTypesupport,
{
    let result = unsafe { rcl_publish(publisher, msg.void_ptr(), std::ptr::null_mut()) };
    if result == RCL_RET_OK as i32 {
        Ok(())
    } else {
        eprintln!("{}", result);
        Err(())
    }
}

pub fn rcl_create_subscription<T>(
    node: &mut rcl_node_t,
    topic: &str,
    callback: Box<dyn FnMut(T) -> ()>,
) -> Result<WrappedSubT<T>, ()>
where
    T: WrappedTypesupport,
{
    let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
    let topic_c_string = CString::new(topic).unwrap();

    let result = unsafe {
        let mut subscription_options = rcl_subscription_get_default_options();
        subscription_options.qos = rmw_qos_profile_t::default();
        rcl_subscription_init(
            &mut subscription_handle,
            node,
            T::get_ts(),
            topic_c_string.as_ptr(),
            &subscription_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        let wrapped_sub = WrappedSubT {
            rcl_handle: subscription_handle,
            rcl_msg: WrappedNativeMsg::<T>::new(),
            callback: callback,
        };

        Ok(wrapped_sub)
    } else {
        eprintln!("{}", result);
        Err(())
    }
}

pub fn rcl_create_subscription_native<T>(
    node: &mut rcl_node_t,
    topic: &str,
    callback: Box<dyn FnMut(&WrappedNativeMsg<T>) -> ()>,
) -> Result<WrappedSubNative<T>, ()>
where
    T: WrappedTypesupport,
{
    let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
    let topic_c_string = CString::new(topic).unwrap();

    let result = unsafe {
        let mut subscription_options = rcl_subscription_get_default_options();
        subscription_options.qos = rmw_qos_profile_t::default();
        rcl_subscription_init(
            &mut subscription_handle,
            node,
            T::get_ts(),
            topic_c_string.as_ptr(),
            &subscription_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        let wrapped_sub = WrappedSubNative {
            rcl_handle: subscription_handle,
            rcl_msg: WrappedNativeMsg::<T>::new(),
            callback: callback,
        };

        Ok(wrapped_sub)
    } else {
        eprintln!("{}", result);
        Err(())
    }
}

pub fn rcl_take_subst(
    ctx: &mut rcl_context_t,
    subs: &mut Vec<Box<dyn Sub>>,
    timeout: i64,
) -> Result<(), ()> {
    let mut ws = unsafe { rcl_get_zero_initialized_wait_set() };

    unsafe {
        rcl_wait_set_init(
            &mut ws,
            subs.len(),
            0,
            0,
            0,
            0,
            0,
            ctx,
            rcutils_get_default_allocator(),
        );
        rcl_wait_set_clear(&mut ws);
    }

    for s in subs.iter() {
        unsafe {
            rcl_wait_set_add_subscription(&mut ws, s.handle(), std::ptr::null_mut());
        }
    }

    unsafe {
        rcl_wait(&mut ws, timeout);
    }

    for s in subs {
        let mut msg_info = rmw_message_info_t::default();

        let ret = unsafe { rcl_take(s.handle(), s.rcl_msg(), &mut msg_info, std::ptr::null_mut()) };

        // fresh message, run cb
        if ret == RCL_RET_OK as i32 {
            s.run_cb();
        }
    }

    unsafe {
        rcl_wait_set_fini(&mut ws);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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

            let memberslice = std::slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);
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
            let msg_native2 = WrappedNativeMsg::<geometry_msgs::msg::AccelWithCovariance>::from(&msg);
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

            let memberslice = std::slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);
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
            let msg_native2 = WrappedNativeMsg::<shape_msgs::msg::SolidPrimitive>::from(&msg);
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
        assert_eq!(msg,msg2);
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
        assert_eq!(msg,new_msg);
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
        assert_ne!(msg,new_msg);
    }

}
