include!(concat!(env!("OUT_DIR"), "/_r2r_generated_msgs.rs"));
include!(concat!(env!("OUT_DIR"), "/_r2r_generated_untyped_helper.rs"));

#[macro_use] extern crate failure_derive;
use serde::{Deserialize, Serialize};
use std::ffi::{CString,CStr};
use std::mem::MaybeUninit;
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
use std::time::Duration;
use std::collections::HashMap;

use msg_gen::*;
use rcl::*;

mod error;
use error::*;

mod utils;
pub use utils::*;

pub type Result<T> = std::result::Result<T, Error>;

pub trait WrappedTypesupport: Serialize + serde::de::DeserializeOwned + Default {
    type CStruct;

    fn get_ts() -> &'static rosidl_message_type_support_t;
    fn create_msg() -> *mut Self::CStruct;
    fn destroy_msg(msg: *mut Self::CStruct);
    fn from_native(msg: &Self::CStruct) -> Self;
    fn copy_to_native(&self, msg: &mut Self::CStruct);
}

pub trait WrappedServiceTypeSupport {
    type Request: WrappedTypesupport;
    type Response: WrappedTypesupport;

    fn get_ts() -> &'static rosidl_service_type_support_t;
}

pub trait WrappedActionTypeSupport {
    type Goal: WrappedTypesupport;
    type Result: WrappedTypesupport;
    type Feedback: WrappedTypesupport;

    fn get_ts() -> &'static rosidl_action_type_support_t;
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

impl<T> Service for WrappedService<T>
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
        // copy native msg to rust type and run callback
        let request = T::Request::from_native(&self.rcl_request_msg);
        let response = (self.callback)(request);
        let mut native_response = WrappedNativeMsg::<T::Response>::from(&response);
        let res = unsafe {
            rcl_send_response(&self.rcl_handle, &mut self.rcl_request, native_response.void_ptr_mut())
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
    rcl_request: rmw_request_id_t,
    // store callbacks with request sequence id and callback function
    callbacks: Vec<(i64, Box<dyn FnOnce(T::Response)>)>,
    rcl_response_msg: WrappedNativeMsg<T::Response>,
}

pub trait Client_ {
    fn handle(&self) -> &rcl_client_t;
    fn run_cb(&mut self) -> ();
    fn rcl_response_msg(&mut self) -> *mut std::os::raw::c_void;
    fn rcl_request_id(&mut self) -> *mut rmw_request_id_t;
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();
}

impl<T> Client_ for WrappedClient<T>
where
    T: WrappedServiceTypeSupport,
{
    fn handle(&self) -> &rcl_client_t {
        &self.rcl_handle
    }

    fn rcl_response_msg(&mut self) -> *mut std::os::raw::c_void {
        self.rcl_response_msg.void_ptr_mut()
    }

    fn rcl_request_id(&mut self) -> *mut rmw_request_id_t {
        &mut self.rcl_request
    }

    fn run_cb(&mut self) -> () {
        // copy native msg to rust type and run callback
        let req_id = self.rcl_request.sequence_number;
        if let Some(idx) = self.callbacks.iter().position(|(id, _)| id == &req_id) {
            let (_, cb_to_run) = self.callbacks.swap_remove(idx);
            let response = T::Response::from_native(&self.rcl_response_msg);
            (cb_to_run)(response);
        } else {
            // I don't think this should be able to occur? Let's panic so we
            // find out...
            let we_have: String = self.callbacks.iter()
                .map(|(id, _)| id.to_string())
                .collect::<Vec<_>>().join(",");
            eprintln!("no such req id: {}, we have [{}], ignoring", req_id, we_have);
        }
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_client_fini(&mut self.rcl_handle, node);
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


#[derive(Debug, Clone)]
pub struct Context {
    context_handle: Arc<Mutex<Box<rcl_context_t>>>,
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
        let args = std::env::args().map(|arg| CString::new(arg).unwrap() ).collect::<Vec<CString>>();
        let mut c_args = args.iter().map(|arg| arg.as_ptr()).collect::<Vec<*const ::std::os::raw::c_char>>();
        c_args.push(std::ptr::null());

        let is_valid = unsafe {
            let allocator = rcutils_get_default_allocator();
            let mut init_options = rcl_get_zero_initialized_init_options();
            rcl_init_options_init(&mut init_options, allocator);
            rcl_init((c_args.len() - 1) as ::std::os::raw::c_int, c_args.as_ptr(), &init_options, ctx.as_mut());
            rcl_init_options_fini(&mut init_options as *mut _);
            rcl_context_is_valid(ctx.as_mut())
        };

        let logging_ok = unsafe {
            let _guard = log_guard();
            let ret = rcl_logging_configure(&ctx.as_ref().global_arguments,
                                            &rcutils_get_default_allocator());
            ret == RCL_RET_OK as i32
        };

        if is_valid && logging_ok {
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
                std::slice::from_raw_parts(
                    (*v.byte_array_value).values,
                    (*v.byte_array_value).size)
            };
            ParameterValue::ByteArray(vals.iter().cloned().collect())
        }
        else if v.bool_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.bool_array_value).values,
                    (*v.bool_array_value).size)
            };
            ParameterValue::BoolArray(vals.iter().cloned().collect())
        }
        else if v.integer_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.integer_array_value).values,
                    (*v.integer_array_value).size)
            };
            ParameterValue::IntegerArray(vals.iter().cloned().collect())
        }
        else if v.double_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.double_array_value).values,
                    (*v.double_array_value).size)
            };
            ParameterValue::DoubleArray(vals.iter().cloned().collect())
        }
        else if v.string_array_value != std::ptr::null_mut() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.string_array_value).data,
                    (*v.string_array_value).size)
            };
            let s = vals.iter().map(|cs| {
                let s = unsafe { CStr::from_ptr(*cs) };
                s.to_str().unwrap_or("").to_owned()
            }).collect();
            ParameterValue::StringArray(s)
        }

        else {
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
    // timers,
    timers: Vec<Timer>,
    // and the publishers, whom we allow to be shared.. hmm.
    pubs: Vec<Arc<rcl_publisher_t>>,
}

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

        let ret = unsafe {
            rcl_arguments_get_param_overrides(&ctx.global_arguments, params.as_mut())
        };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not read parameters: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        if *params == std::ptr::null_mut() {
            return Ok(());
        }

        let node_names = unsafe {
            std::slice::from_raw_parts(
                (*(*params.as_ref())).node_names, (*(*params.as_ref())).num_nodes)
        };

        let node_params = unsafe {
            std::slice::from_raw_parts(
                (*(*params.as_ref())).params, (*(*params.as_ref())).num_nodes)
        };

        let qualified_name = self.fully_qualified_name()?;
        let name = self.name()?;

        for (nn, np) in node_names.iter().zip(node_params) {
            let node_name_cstr = unsafe { CStr::from_ptr(*nn) };
            let node_name = node_name_cstr.to_str().unwrap_or("");

            // This is copied from rclcpp, but there is a comment there suggesting
            // that more wildcards will be added in the future. Take note and mimic
            // their behavior.
            if !(
                node_name == "/**" ||
                node_name == "**" ||
                qualified_name == node_name ||
                name == node_name
            ) {
                continue;
            }

            // make key value pairs.
            let param_names = unsafe {
                std::slice::from_raw_parts(np.parameter_names,np.num_params)
            };

            let param_values = unsafe {
                std::slice::from_raw_parts(np.parameter_values,np.num_params)
            };

            for (s,v) in param_names.iter().zip(param_values) {
                let s = unsafe { CStr::from_ptr(*s) };
                let key = s.to_str().unwrap_or("");
                let val = ParameterValue::from_rcl(&*v);
                self.params.insert(key.to_owned(), val);
            }
        }

        unsafe { rcl_yaml_node_struct_fini(*params) } ;
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

    pub fn create_service_helper(&mut self, service_name: &str,
                                 service_ts: *const rosidl_service_type_support_t)
                                 -> Result<rcl_service_t> {
        let mut service_handle = unsafe { rcl_get_zero_initialized_service() };
        let service_name_c_string = CString::new(service_name)
            .map_err(|_|Error::RCL_RET_INVALID_ARGUMENT)?;

        let result = unsafe {
            let service_options = rcl_service_get_default_options();
            rcl_service_init(&mut service_handle, self.node_handle.as_mut(),
                             service_ts, service_name_c_string.as_ptr(), &service_options)
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
            callback: callback,
        };

        self.services.push(Box::new(ws));
        Ok(self.services.last().unwrap().handle()) // hmm...
    }


    pub fn create_client_helper(&mut self, service_name: &str,
                                service_ts: *const rosidl_service_type_support_t)
                                -> Result<rcl_client_t> {
        let mut client_handle = unsafe { rcl_get_zero_initialized_client() };
        let service_name_c_string = CString::new(service_name)
            .map_err(|_|Error::RCL_RET_INVALID_ARGUMENT)?;

        let result = unsafe {
            let client_options = rcl_client_get_default_options();
            rcl_client_init(&mut client_handle, self.node_handle.as_mut(),
                             service_ts, service_name_c_string.as_ptr(), &client_options)
        };
        if result == RCL_RET_OK as i32 {
            Ok(client_handle)
        } else {
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn create_client<T: 'static>(
        &mut self,
        service_name: &str,
    ) -> Result<Client<T>>
    where
        T: WrappedServiceTypeSupport,
    {
        let client_handle = self.create_client_helper(service_name, T::get_ts())?;
        let cloned_ch = rcl_client_t {
            impl_: client_handle.impl_
        };
        let ws = WrappedClient::<T> {
            rcl_handle: cloned_ch,
            rcl_request: rmw_request_id_t {
                writer_guid: [0; 16usize],
                sequence_number: 0,
            },
            rcl_response_msg: WrappedNativeMsg::<T::Response>::new(),
            callbacks: Vec::new(),
        };

        let arc = Arc::new(Mutex::new(ws));
        let client_ = Arc::downgrade(&arc);
        self.clients.push((client_handle, arc));
        let c = Client {
            client_
        };
        Ok(c)
    }

    pub fn service_available<T: WrappedServiceTypeSupport>(&self, client: &Client<T>) -> Result<bool> {
        let client = client.client_.upgrade().ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let client = client.lock().unwrap();
        let mut avail = false;
        let result = unsafe {
            rcl_service_server_is_available(
                self.node_handle.as_ref(),
                client.handle(),
                &mut avail)
        };

        if result == RCL_RET_OK as i32 {
            Ok(avail)
        } else {
            eprintln!("coult not send request {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn create_publisher_helper(&mut self, topic: &str,
                                   typesupport: *const rosidl_message_type_support_t) -> Result<rcl_publisher_t> {
        let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
        let topic_c_string = CString::new(topic)
            .map_err(|_|Error::RCL_RET_INVALID_ARGUMENT)?;

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
                    self.timers.len(),
                    self.clients.len(),
                    self.services.len(),
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

        let ret = unsafe {
            rcl_wait(&mut ws, timeout)
        };

        if ret == RCL_RET_TIMEOUT as i32 {
            unsafe {
                rcl_wait_set_fini(&mut ws);
            }
            return;
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

        let ws_timers =
            unsafe { std::slice::from_raw_parts(ws.timers, ws.size_of_timers) };
        assert_eq!(ws_timers.len(), self.timers.len());
        for (s, ws_s) in self.timers.iter_mut().zip(ws_timers) {
            if ws_s != &std::ptr::null() {
                let mut is_ready = false;
                let ret = unsafe {
                    rcl_timer_is_ready(&s.timer_handle, &mut is_ready)
                };
                if ret == RCL_RET_OK as i32 {
                    if is_ready {
                        let mut nanos = 0i64;
                        // todo: error handling
                        let ret = unsafe { rcl_timer_get_time_since_last_call(&s.timer_handle,
                                                                              &mut nanos) };
                        if ret == RCL_RET_OK as i32 {
                            let ret = unsafe { rcl_timer_call(&mut s.timer_handle) };
                            if ret == RCL_RET_OK as i32 {
                                (s.callback)(Duration::from_nanos(nanos as u64));
                            }
                        }
                    }
                }
            }
        }

        let ws_clients =
            unsafe { std::slice::from_raw_parts(ws.clients, ws.size_of_clients) };
        assert_eq!(ws_clients.len(), self.clients.len());
        for ((_, s), ws_s) in self.clients.iter_mut().zip(ws_clients) {
            if ws_s != &std::ptr::null() {
                let mut s = s.lock().unwrap();
                let ret = unsafe {
                    rcl_take_response(s.handle(), s.rcl_request_id(), s.rcl_response_msg())
                };
                if ret == RCL_RET_OK as i32 {
                    s.run_cb();
                }
            }
        }

        let ws_services =
            unsafe { std::slice::from_raw_parts(ws.services, ws.size_of_services) };
        assert_eq!(ws_services.len(), self.services.len());
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

    pub fn create_wall_timer(
        &mut self,
        period: Duration,
        callback: Box<dyn FnMut(Duration) -> ()>) -> Result<&Timer> {

        let mut clock_handle = MaybeUninit::<rcl_clock_t>::uninit();

        let ret = unsafe {
            rcl_steady_clock_init(clock_handle.as_mut_ptr(), &mut rcutils_get_default_allocator())
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
                rcl_timer_init(&mut timer_handle, clock_handle.as_mut(),
                               ctx.as_mut(), period.as_nanos() as i64,
                               None, rcutils_get_default_allocator())
            };

            if ret != RCL_RET_OK as i32 {
                eprintln!("could not create timer: {}", ret);
                return Err(Error::from_rcl_error(ret));
            }
        }

        let timer = Timer { timer_handle, clock_handle, callback };
        self.timers.push(timer);

        Ok(&self.timers[self.timers.len()-1])
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

pub struct Timer {
    timer_handle: rcl_timer_t,
    clock_handle: Box<rcl_clock_t>,
    callback: Box<dyn FnMut(Duration) -> ()>,
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


impl<T> Client<T>
where
    T: WrappedServiceTypeSupport,
{
    pub fn request(&self, msg: &T::Request, cb: Box<dyn FnOnce(T::Response) -> ()>) -> Result<()>
    where
        T: WrappedServiceTypeSupport,
    {
        // upgrade to actual ref. if still alive
        let client = self.client_.upgrade().ok_or(Error::RCL_RET_CLIENT_INVALID)?;
        let mut client = client.lock().unwrap();
        // copy rust msg to native and publish it
        let native_msg: WrappedNativeMsg<T::Request> = WrappedNativeMsg::<T::Request>::from(msg);
        let mut seq_no = 0i64;
        let result = unsafe {
            rcl_send_request(
                &client.rcl_handle,
                native_msg.void_ptr(),
                &mut seq_no,
            )
        };

        if result == RCL_RET_OK as i32 {
            client.callbacks.push((seq_no, cb));
            Ok(())
        } else {
            eprintln!("coult not send request {}", result);
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
            rcl_clock_init(rcl_ct, clock_handle.as_mut_ptr(), &mut rcutils_get_default_allocator())
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
        let ret = unsafe {
            rcl_clock_get_now(&mut *self.clock_handle,
                              &mut tp)
        };

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
        builtin_interfaces::msg::Time {
            sec,
            nanosec,
        }
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
        res.sequence = vec![1,2,3];
        let rn = WrappedNativeMsg::<_>::from(&res);
        let res2 = Fibonacci::Result::from_native(&rn);
        println!("res2 {:?}", res2);
        assert_eq!(res, res2);

        let mut fb = Fibonacci::Feedback::default();
        fb.sequence = vec![4,3,6];
        let fbn = WrappedNativeMsg::<_>::from(&fb);
        let fb2 = Fibonacci::Feedback::from_native(&fbn);
        println!("feedback2 {:?}", fb2);
        assert_eq!(fb, fb2);
    }
}
