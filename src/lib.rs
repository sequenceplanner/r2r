include!(concat!(env!("OUT_DIR"), "/generated_msgs.rs"));
include!(concat!(env!("OUT_DIR"), "/generated_untyped_helper.rs"));

#[macro_use] extern crate failure_derive;
use serde::{Deserialize, Serialize};
use std::ffi::{CString,CStr};
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
use std::time::Duration;
use std::collections::HashMap;

use msg_gen::*;
use rcl::*;

mod error;
use error::*;

pub type Result<T> = std::result::Result<T, Error>;

pub trait WrappedTypesupport: Serialize + serde::de::DeserializeOwned  {
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

#[derive(Debug)]
pub struct WrappedNativeMsgUntyped {
    ts: &'static rosidl_message_type_support_t,
    msg: *mut std::os::raw::c_void,
    destroy: fn(*mut std::os::raw::c_void),
    msg_to_json: fn(native: *const std::os::raw::c_void) ->
        std::result::Result<serde_json::Value, serde_json::error::Error>,
    msg_from_json: fn(native: *mut std::os::raw::c_void, json: serde_json::Value) ->
        std::result::Result<(), serde_json::error::Error>,
}

impl WrappedNativeMsgUntyped {
    fn new<T>() -> Self where T: WrappedTypesupport {
        let destroy = | native: *mut std::os::raw::c_void | {
            let native_msg = native as *mut T::CStruct;
            T::destroy_msg(native_msg);
        };

        let msg_to_json = | native: *const std::os::raw::c_void | {
            let msg = unsafe { T::from_native(&*(native as *const T::CStruct)) };
            serde_json::to_value(&msg)
        };

        let msg_from_json = | native: *mut std::os::raw::c_void, json: serde_json::Value | {
            serde_json::from_value(json).map(|msg: T| {
                unsafe { msg.copy_to_native(&mut *(native as *mut T::CStruct)); }
            })
        };

        WrappedNativeMsgUntyped {
            ts: T::get_ts(),
            msg: T::create_msg() as *mut std::os::raw::c_void,
            destroy: destroy,
            msg_to_json: msg_to_json,
            msg_from_json: msg_from_json,
        }
    }

    fn to_json(&self) -> Result<serde_json::Value> {
        let json = (self.msg_to_json)(self.msg);
        json.map_err(|serde_err|Error::SerdeError { err: serde_err.to_string() })
    }

    fn from_json(&mut self, json: serde_json::Value) -> Result<()> {
        (self.msg_from_json)(self.msg, json).
            map_err(|serde_err|Error::SerdeError { err: serde_err.to_string() })
    }

    pub fn void_ptr(&self) -> *const std::os::raw::c_void {
        self.msg as *const _ as *const std::os::raw::c_void
    }

    pub fn void_ptr_mut(&mut self) -> *mut std::os::raw::c_void {
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
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();
}

struct WrappedSub<T>
where
    T: WrappedTypesupport,
{
    rcl_handle: rcl_subscription_t,
    callback: Box<dyn FnMut(T) -> ()>,
    rcl_msg: WrappedNativeMsg<T>,
}

struct WrappedSubNative<T>
where
    T: WrappedTypesupport,
{
    rcl_handle: rcl_subscription_t,
    callback: Box<dyn FnMut(&WrappedNativeMsg<T>) -> ()>,
    rcl_msg: WrappedNativeMsg<T>,
}

struct WrappedSubUntyped {
    rcl_handle: rcl_subscription_t,
    rcl_msg: WrappedNativeMsgUntyped,
    callback: Box<dyn FnMut(Result<serde_json::Value>) -> ()>,
}

impl<T> Sub for WrappedSub<T>
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

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
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

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl Sub for WrappedSubUntyped
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn rcl_msg(&mut self) -> *mut std::os::raw::c_void {
        self.rcl_msg.void_ptr_mut()
    }

    fn run_cb(&mut self) -> () {
        let json = self.rcl_msg.to_json();
        (self.callback)(json);
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
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
pub struct Publisher<T>
where
    T: WrappedTypesupport,
{
    handle: Weak<rcl_publisher_t>,
    type_: PhantomData<T>,
}

unsafe impl Send for PublisherUntyped {}
pub struct PublisherUntyped {
    handle: Weak<rcl_publisher_t>,
    type_: String,
}

#[derive(Debug, Clone)]
pub struct Context {
    context_handle: Arc<Mutex<Box<rcl_context_t>>>,
}

// Not 100% about this one. From our end the context is rarely used and can be locked by a mutex for that. But I haven't investigated if its use is thread-safe between nodes. May remove send here later.
unsafe impl Send for Context {}

impl Context {
    pub fn create() -> Result<Context> {
        let mut ctx: Box<rcl_context_t> = unsafe { Box::new(rcl_get_zero_initialized_context()) };
        let is_valid = unsafe {
            let allocator = rcutils_get_default_allocator();
            let mut init_options = rcl_get_zero_initialized_init_options();
            rcl_init_options_init(&mut init_options, allocator);
            rcl_init(0, std::ptr::null(), &init_options, ctx.as_mut());
            rcl_init_options_fini(&mut init_options as *mut _);
            rcl_context_is_valid(ctx.as_mut())
        };

        if is_valid {
            Ok(Context {
                context_handle: Arc::new(Mutex::new(ctx)),
            })
        } else {
            Err(Error::RCL_RET_ERROR) // TODO
        }
    }
}

#[derive(Debug, Clone)]
pub struct ContextHandle(Arc<Mutex<Box<rcl_context_t>>>);

impl Drop for ContextHandle {
    fn drop(&mut self) {
        println!("DROPPING CONTEXT HANDLE!");
        let mut ctx_handle = self.0.lock().unwrap();
        // TODO: error handling? atleast probably need rcl_reset_error
        unsafe {
            rcl::rcl_shutdown(ctx_handle.as_mut());
            rcl::rcl_context_fini(ctx_handle.as_mut());
        }
    }
}

pub struct Node {
    context: Context,
    node_handle: Box<rcl_node_t>,
    // the node owns the subscribers
    subs: Vec<Box<dyn Sub>>,
    // and the publishers, whom we allow to be shared.. hmm.
    pubs: Vec<Arc<rcl_publisher_t>>,
}

impl Node {
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
            Ok(Node {
                context: ctx,
                node_handle: node_handle,
                subs: Vec::new(),
                pubs: Vec::new(),
            })
        } else {
            eprintln!("could not create node{}", res);
            Err(Error::from_rcl_error(res))
        }
    }

    fn create_subscription_helper(&mut self, topic: &str, ts: *const rosidl_message_type_support_t) -> Result<rcl_subscription_t> {
        let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
        let topic_c_string = CString::new(topic).map_err(|_|Error::RCL_RET_INVALID_ARGUMENT)?;

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

    pub fn subscribe<T: 'static>(
        &mut self,
        topic: &str,
        callback: Box<dyn FnMut(T) -> ()>,
    ) -> Result<&rcl_subscription_t>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle = self.create_subscription_helper(topic, T::get_ts())?;
        let ws = WrappedSub {
            rcl_handle: subscription_handle,
            rcl_msg: WrappedNativeMsg::<T>::new(),
            callback: callback,
        };
        self.subs.push(Box::new(ws));
        Ok(self.subs.last().unwrap().handle()) // hmm...
    }

    pub fn subscribe_native<T: 'static>(
        &mut self,
        topic: &str,
        callback: Box<dyn FnMut(&WrappedNativeMsg<T>) -> ()>,
    ) -> Result<&rcl_subscription_t>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle = self.create_subscription_helper(topic, T::get_ts())?;
        let ws = WrappedSubNative {
            rcl_handle: subscription_handle,
            rcl_msg: WrappedNativeMsg::<T>::new(),
            callback: callback,
        };
        self.subs.push(Box::new(ws));
        Ok(self.subs.last().unwrap().handle()) // hmm...
    }

    // Its not really untyped since we know the underlying type... But we throw this info away :)
    pub fn subscribe_untyped(
        &mut self,
        topic: &str,
        topic_type: &str,
        callback: Box<dyn FnMut(Result<serde_json::Value>) -> ()>,
    ) -> Result<&rcl_subscription_t> {
        let msg = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let subscription_handle = self.create_subscription_helper(topic, msg.ts)?;

        let ws = WrappedSubUntyped {
            rcl_handle: subscription_handle,
            rcl_msg: msg,
            callback: callback,
        };
        self.subs.push(Box::new(ws));
        Ok(self.subs.last().unwrap().handle()) // hmm...
    }


    pub fn create_publisher_helper(&mut self, topic: &str,
                                   typesupport: *const rosidl_message_type_support_t) -> Result<rcl_publisher_t> {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let topic_c_string = CString::new(topic).map_err(|_|Error::RCL_RET_INVALID_ARGUMENT)?;

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

    pub fn create_publisher_untyped(&mut self, topic: &str, topic_type: &str) -> Result<PublisherUntyped> {
        let dummy = WrappedNativeMsgUntyped::new_from(topic_type)?; // TODO, get ts without allocating msg
        let publisher_handle = self.create_publisher_helper(topic, dummy.ts)?;

        self.pubs.push(Arc::new(publisher_handle));
        let p = PublisherUntyped {
            handle: Arc::downgrade(self.pubs.last().unwrap()),
            type_: topic_type.to_owned(),
        };
        Ok(p)
    }

    pub fn spin_once(&mut self, timeout: Duration) {
        let timeout = timeout.as_nanos() as i64;
        let mut ws = unsafe { rcl_get_zero_initialized_wait_set() };

        // #[doc = "* This function is thread-safe for unique wait sets with unique contents."]
        // #[doc = "* This function cannot operate on the same wait set in multiple threads, and"]
        // #[doc = "* the wait sets may not share content."]
        // #[doc = "* For example, calling rcl_wait() in two threads on two different wait sets"]
        // #[doc = "* that both contain a single, shared guard condition is undefined behavior."]

        {
            let mut ctx = self.context.context_handle.lock().unwrap();

            unsafe {
                rcl_wait_set_init(
                    &mut ws,
                    self.subs.len(),
                    0,
                    0,
                    0,
                    0,
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

        unsafe {
            rcl_wait(&mut ws, timeout);
        }

        let ws_subs =
            unsafe { std::slice::from_raw_parts(ws.subscriptions, ws.size_of_subscriptions) };
        assert_eq!(ws_subs.len(), self.subs.len());
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        for (s, ws_s) in self.subs.iter_mut().zip(ws_subs) {
            if ws_s != &std::ptr::null() {
                let ret = unsafe {
                    rcl_take(s.handle(), s.rcl_msg(), &mut msg_info, std::ptr::null_mut())
                };
                if ret == RCL_RET_OK as i32 {
                    s.run_cb();
                }
            }
        }

        unsafe {
            rcl_wait_set_fini(&mut ws);
        }
    }


    pub fn get_topic_names_and_types(&self) -> Result<HashMap<String, Vec<String>>> {
        let mut tnat = unsafe { rmw_get_zero_initialized_names_and_types() };
        let ret = unsafe {
            rcl_get_topic_names_and_types(self.node_handle.as_ref(), &mut rcutils_get_default_allocator(), false, &mut tnat)
        };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not get topic names and types {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        let names = unsafe { std::slice::from_raw_parts(tnat.names.data, tnat.names.size) };
        let types = unsafe { std::slice::from_raw_parts(tnat.types, tnat.names.size) };

        let mut res = HashMap::new();
        for (n,t) in names.iter().zip(types) {
            let topic_name = unsafe {CStr::from_ptr(*n).to_str().unwrap().to_owned() };
            let topic_types = unsafe { std::slice::from_raw_parts(t, t.size) };
            let topic_types: Vec<String> = unsafe {
                topic_types.iter().map(|t| CStr::from_ptr(*((*t).data)).to_str().unwrap().to_owned()).collect()
            };
            res.insert(topic_name, topic_types);
        }
        unsafe { rmw_names_and_types_fini(&mut tnat); } // TODO: check return value
        Ok(res)
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
        for s in &mut self.subs {
            s.destroy(&mut self.node_handle);
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

impl<T> Publisher<T>
where
    T: WrappedTypesupport,
{
    pub fn publish(&self, msg: &T) -> Result<()>
    where
        T: WrappedTypesupport,
    {
        // upgrade to actual ref. if still alive
        let publisher = self.handle.upgrade().ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;
        // copy rust msg to native and publish it
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
        let publisher = self.handle.upgrade().ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

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

impl PublisherUntyped {
    pub fn publish(&self, msg: serde_json::Value) -> Result<()> {
        // upgrade to actual ref. if still alive
        let publisher = self.handle.upgrade().ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

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

        let mut native = WrappedNativeMsgUntyped::new_from("trajectory_msgs/msg/JointTrajectoryPoint").unwrap();
        native.from_json(json.clone()).unwrap();
        let json2 = native.to_json().unwrap();
        assert_eq!(json, json2);

        let msg2: trajectory_msgs::msg::JointTrajectoryPoint = serde_json::from_value(json2).unwrap();
        assert_eq!(msg, msg2);
    }

    #[cfg(r2r__test_msgs__msg__Arrays)]
    #[test]
    fn test_test_msgs_array() -> () {
        let mut msg = test_msgs::msg::Arrays::default();
        println!("msg: {:?}", msg.string_values);
        msg.string_values = vec![
            "hej".to_string(), "hopp".to_string(), "stropp".to_string()
        ];

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
        msg.string_values = vec![ "hej".to_string(), "hopp".to_string() ];
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

}
