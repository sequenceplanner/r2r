use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::fmt::Debug;
use std::marker::PhantomData;
use std::mem::MaybeUninit;
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use futures::channel::{mpsc, oneshot};
use futures::future::FutureExt;
use futures::future::TryFutureExt;
use futures::stream::{Stream, StreamExt};
use std::future::Future;

use retain_mut::RetainMut;

// otherwise crates using r2r needs to specify the same version of uuid as
// this crate depend on, which seem like bad user experience.
pub extern crate uuid;

use actions::*;
use rcl::*;

mod error;
use error::*;

mod msg_types;
use msg_types::*;

pub use msg_types::generated_msgs::*;
pub use msg_types::WrappedNativeMsg as NativeMsg;

mod utils;
pub use utils::*;

mod subscribers;
use subscribers::*;

mod publishers;
use publishers::*;

mod services;
use services::*;

mod clients;
use clients::*;
pub use clients::{Client, UntypedClient};

mod action_clients;
use action_clients::*;

mod action_servers;
use action_servers::*;

pub use action_servers::ServerGoal;

mod context;
pub use context::Context;

mod parameters;
use parameters::*;
pub use parameters::ParameterValue;

mod clocks;
pub use clocks::{Clock, ClockType};

/// Encapsulates a service request. In contrast to having a simply callback from
/// Request -> Response types that is called synchronously, the service request
/// can be moved around and completed asynchronously.
pub struct ServiceRequest<T>
where
    T: WrappedServiceTypeSupport,
{
    pub message: T::Request,
    request_id: rmw_request_id_t,
    response_sender: oneshot::Sender<(rmw_request_id_t, T::Response)>,
}

impl<T> ServiceRequest<T>
where
    T: WrappedServiceTypeSupport,
{
    /// Complete the service request, consuming the request in the process.
    /// The reply is sent back on the next "ros spin".
    pub fn respond(self, msg: T::Response) {
        match self.response_sender.send((self.request_id, msg)) {
            Err(_) => {
                println!("service response receiver dropped");
            }
            _ => {}
        }
    }
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

pub struct Node {
    context: Context,
    pub params: HashMap<String, ParameterValue>,
    node_handle: Box<rcl_node_t>,
    // the node owns the subscribers
    subs: Vec<Box<dyn Subscriber_>>,
    // services,
    services: Vec<Box<dyn Service_>>,
    // service clients
    clients: Vec<Arc<Mutex<dyn Client_>>>,
    // action clients
    action_clients: Vec<Arc<Mutex<dyn ActionClient_>>>,
    // action servers
    action_servers: Vec<Arc<Mutex<dyn ActionServer_>>>,
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
                let val = parameter_value_from_rcl(&*v);
                self.params.insert(key.to_owned(), val);
            }
        }

        unsafe { rcl_yaml_node_struct_fini(*params) };
        Ok(())
    }

    pub fn create(ctx: Context, name: &str, namespace: &str) -> Result<Node> {
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
                node_handle,
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

    pub fn subscribe<T: 'static>(&mut self, topic: &str) -> Result<impl Stream<Item = T> + Unpin>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle = create_subscription_helper(self.node_handle.as_mut(), topic, T::get_ts())?;
        let (sender, receiver) = mpsc::channel::<T>(10);

        let ws = TypedSubscriber {
            rcl_handle: subscription_handle,
            sender,
        };
        self.subs.push(Box::new(ws));
        Ok(receiver)
    }

    pub fn subscribe_native<T: 'static>(
        &mut self,
        topic: &str,
    ) -> Result<impl Stream<Item = WrappedNativeMsg<T>> + Unpin>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle = create_subscription_helper(self.node_handle.as_mut(), topic, T::get_ts())?;
        let (sender, receiver) = mpsc::channel::<WrappedNativeMsg<T>>(10);

        let ws = NativeSubscriber {
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
    ) -> Result<impl Stream<Item = Result<serde_json::Value>> + Unpin> {
        let msg = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let subscription_handle = create_subscription_helper(self.node_handle.as_mut(), topic, msg.ts)?;
        let (sender, receiver) = mpsc::channel::<Result<serde_json::Value>>(10);

        let ws = UntypedSubscriber {
            rcl_handle: subscription_handle,
            topic_type: topic_type.to_string(),
            sender,
        };
        self.subs.push(Box::new(ws));
        Ok(receiver)
    }

    fn create_service_helper(
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
    ) -> Result<impl Stream<Item = ServiceRequest<T>> + Unpin>
    where
        T: WrappedServiceTypeSupport,
    {
        let service_handle = self.create_service_helper(service_name, T::get_ts())?;
        let (sender, receiver) = mpsc::channel::<ServiceRequest<T>>(10);

        let ws = TypedService::<T> {
            rcl_handle: service_handle,
            outstanding_requests: vec![],
            sender,
        };

        self.services.push(Box::new(ws));
        Ok(receiver)
    }

    pub fn create_client<T: 'static>(&mut self, service_name: &str) -> Result<Client<T>>
    where
        T: WrappedServiceTypeSupport,
    {
        let client_handle = create_client_helper(self.node_handle.as_mut(), service_name, T::get_ts())?;
        let ws = TypedClient::<T> {
            rcl_handle: client_handle,
            response_channels: Vec::new(),
        };

        let client_arc = Arc::new(Mutex::new(ws));
        let c = make_client(Arc::downgrade(&client_arc));
        self.clients.push(client_arc);
        Ok(c)
    }

    /// Create a service client without having the concrete rust type.
    pub fn create_client_untyped(&mut self, service_name: &str, service_type: &str) -> Result<UntypedClient>
    {
        let service_type = UntypedServiceSupport::new_from(service_type)?;
        let client_handle = create_client_helper(self.node_handle.as_mut(), service_name, service_type.ts)?;
        let client = UntypedClient_ {
            service_type,
            rcl_handle: client_handle,
            response_channels: Vec::new(),
        };

        let client_arc = Arc::new(Mutex::new(client));
        let c = make_untyped_client(Arc::downgrade(&client_arc));
        self.clients.push(client_arc);
        Ok(c)
    }

    pub fn service_available<T: 'static + WrappedServiceTypeSupport>(&mut self, client: &Client<T>) -> Result<bool> {
        service_available(self.node_handle.as_mut(), client)
    }

    pub fn service_available_untyped(&mut self, client: &UntypedClient) -> Result<bool> {
        service_available_untyped(self.node_handle.as_mut(), client)
    }

    fn create_action_client_helper(
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
        let client = WrappedActionClient::<T> {
            rcl_handle: client_handle,
            goal_response_channels: Vec::new(),
            cancel_response_channels: Vec::new(),
            feedback_senders: Vec::new(),
            result_senders: Vec::new(),
            result_requests: Vec::new(),
            goal_status: HashMap::new(),
        };

        let client_arc = Arc::new(Mutex::new(client));
        self.action_clients.push(client_arc.clone());
        let c = ActionClient { client: Arc::downgrade(&client_arc) };
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

    fn create_action_server_helper(
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
        let server = WrappedActionServer::<T> {
            rcl_handle: server_handle,
            clock_handle,
            accept_goal_cb,
            accept_cancel_cb,
            goal_cb,
            goals: HashMap::new(),
            result_msgs: HashMap::new(),
            result_requests: HashMap::new(),
        };

        let server_arc = Arc::new(Mutex::new(server));
        self.action_servers.push(server_arc.clone());
        let c = ActionServer { server: Arc::downgrade(&server_arc) };
        Ok(c)
    }

    pub fn create_publisher<T>(&mut self, topic: &str) -> Result<Publisher<T>>
    where
        T: WrappedTypesupport,
    {
        let publisher_handle = create_publisher_helper(self.node_handle.as_mut(), topic, T::get_ts())?;
        let arc = Arc::new(publisher_handle);
        let p = make_publisher(Arc::downgrade(&arc));
        self.pubs.push(arc);
        Ok(p)
    }

    pub fn create_publisher_untyped(
        &mut self,
        topic: &str,
        topic_type: &str,
    ) -> Result<PublisherUntyped> {
        let dummy = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let publisher_handle = create_publisher_helper(self.node_handle.as_mut(), topic, dummy.ts)?;
        let arc = Arc::new(publisher_handle);
        let p = make_publisher_untyped(Arc::downgrade(&arc), topic_type.to_owned());
        self.pubs.push(arc);
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
        // first handle any completed service responses
        for s in &mut self.services {
            s.send_completed_responses();
        }

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
        for c in &self.action_clients {
            let mut num_subs = 0;
            let mut num_gc = 0;
            let mut num_timers = 0;
            let mut num_clients = 0;
            let mut num_services = 0;

            Self::action_client_get_num_waits(
                c.lock().unwrap().handle(),
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
        for s in &self.action_servers {
            let mut num_subs = 0;
            let mut num_gc = 0;
            let mut num_timers = 0;
            let mut num_clients = 0;
            let mut num_services = 0;

            Self::action_server_get_num_waits(
                s.lock().unwrap().handle(),
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

        for s in &self.subs {
            unsafe {
                rcl_wait_set_add_subscription(&mut ws, s.handle(), std::ptr::null_mut());
            }
        }

        for s in &self.timers {
            unsafe {
                rcl_wait_set_add_timer(&mut ws, &s.timer_handle, std::ptr::null_mut());
            }
        }

        for s in &self.clients {
            unsafe {
                rcl_wait_set_add_client(&mut ws, s.lock().unwrap().handle(), std::ptr::null_mut());
            }
        }

        for s in &self.services {
            unsafe {
                rcl_wait_set_add_service(&mut ws, s.handle(), std::ptr::null_mut());
            }
        }

        // code (further) below assumes that actions are added last... perhaps a
        // bad assumption.  e.g. we add subscriptions and timers of
        // the node before ones created automatically by actions. we
        // then assume that we can count on the waitables created by
        // the actions are added at the end of the wait set arrays
        for ac in &self.action_clients {
            unsafe {
                rcl_action_wait_set_add_action_client(
                    &mut ws,
                    ac.lock().unwrap().handle(),
                    std::ptr::null_mut(),
                    std::ptr::null_mut(),
                );
            }
        }
        for acs in &self.action_servers {
            unsafe {
                rcl_action_wait_set_add_action_server(&mut ws, acs.lock().unwrap().handle(), std::ptr::null_mut());
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
        for (s, ws_s) in self.clients.iter_mut().zip(ws_clients) {
            if ws_s != &std::ptr::null() {
                let mut s = s.lock().unwrap();
                s.handle_response();
            }
        }

        let ws_services = unsafe { std::slice::from_raw_parts(ws.services, self.services.len()) };
        for (s, ws_s) in self.services.iter_mut().zip(ws_services) {
            if ws_s != &std::ptr::null() {
                s.handle_request();
            }
        }

        for ac in &self.action_clients {
            let mut is_feedback_ready = false;
            let mut is_status_ready = false;
            let mut is_goal_response_ready = false;
            let mut is_cancel_response_ready = false;
            let mut is_result_response_ready = false;

            let ret = unsafe {
                rcl_action_client_wait_set_get_entities_ready(
                    &ws,
                    ac.lock().unwrap().handle(),
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
                let mut acs = ac.lock().unwrap();
                acs.handle_feedback_msg();
            }

            if is_status_ready {
                let mut acs = ac.lock().unwrap();
                acs.handle_status_msg();
            }

            if is_goal_response_ready {
                let mut acs = ac.lock().unwrap();
                acs.handle_goal_response();
            }

            if is_cancel_response_ready {
                let mut acs = ac.lock().unwrap();
                acs.handle_cancel_response();
            }

            if is_result_response_ready {
                let mut acs = ac.lock().unwrap();
                acs.handle_result_response();
            }
        }

        for s in &self.action_servers {
            let mut is_goal_request_ready = false;
            let mut is_cancel_request_ready = false;
            let mut is_result_request_ready = false;
            let mut is_goal_expired = false;

            let ret = unsafe {
                rcl_action_server_wait_set_get_entities_ready(
                    &ws,
                    s.lock().unwrap().handle(),
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

struct Timer_ {
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
        for c in &mut self.action_clients {
            c.lock().unwrap().destroy(&mut self.node_handle);
        }
        for s in &mut self.action_servers {
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
}
