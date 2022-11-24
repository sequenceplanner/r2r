use futures::channel::{mpsc, oneshot};
use futures::future::FutureExt;
use futures::future::TryFutureExt;
use futures::future::{self, join_all};
use futures::stream::{Stream, StreamExt};
use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::future::Future;
use std::mem::MaybeUninit;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use r2r_actions::*;
use r2r_rcl::*;

use crate::action_clients::*;
use crate::action_clients_untyped::*;
use crate::action_servers::*;
use crate::clients::*;
use crate::clocks::*;
use crate::context::*;
use crate::error::*;
use crate::msg_types::generated_msgs::rcl_interfaces;
use crate::msg_types::*;
use crate::parameters::*;
use crate::publishers::*;
use crate::qos::QosProfile;
use crate::services::*;
use crate::subscribers::*;

/// A ROS Node.
///
/// This struct owns all subscribes, publishers, etc.  To get events
/// from the ROS network into your ros application, `spin_once` should
/// be called continously.
pub struct Node {
    context: Context,
    /// ROS parameter values.
    pub params: Arc<Mutex<HashMap<String, ParameterValue>>>,
    node_handle: Box<rcl_node_t>,
    // the node owns the subscribers
    subscribers: Vec<Box<dyn Subscriber_>>,
    // services,
    services: Vec<Arc<Mutex<dyn Service_>>>,
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
    /// Returns the name of the node.
    pub fn name(&self) -> Result<String> {
        let cstr = unsafe { rcl_node_get_name(self.node_handle.as_ref()) };
        if cstr.is_null() {
            return Err(Error::RCL_RET_NODE_INVALID);
        }
        let s = unsafe { CStr::from_ptr(cstr) };
        Ok(s.to_str().unwrap_or("").to_owned())
    }

    /// Returns the fully qualified name of the node.
    pub fn fully_qualified_name(&self) -> Result<String> {
        let cstr = unsafe { rcl_node_get_fully_qualified_name(self.node_handle.as_ref()) };
        if cstr.is_null() {
            return Err(Error::RCL_RET_NODE_INVALID);
        }
        let s = unsafe { CStr::from_ptr(cstr) };
        Ok(s.to_str().unwrap_or("").to_owned())
    }

    /// Returns the namespace of the node.
    pub fn namespace(&self) -> Result<String> {
        let cstr = unsafe { rcl_node_get_namespace(self.node_handle.as_ref()) };
        if cstr.is_null() {
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

        if params.is_null() {
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

            let mut params = self.params.lock().unwrap();
            for (s, v) in param_names.iter().zip(param_values) {
                let s = unsafe { CStr::from_ptr(*s) };
                let key = s.to_str().unwrap_or("");
                let val = ParameterValue::from_rcl(&*v);
                params.insert(key.to_owned(), val);
            }
        }

        unsafe { rcl_yaml_node_struct_fini(*params) };
        Ok(())
    }

    /// Creates a ROS node.
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
                params: Arc::new(Mutex::new(HashMap::new())),
                context: ctx,
                node_handle,
                subscribers: Vec::new(),
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

    /// Creates parameter service handlers for the Node.
    ///
    /// This function returns a tuple (`Future`, `Stream`), where the
    /// future should be spawned on onto the executor of choice. The
    /// `Stream` produces events whenever parameters change from
    /// external sources. The event elements of the event stream
    /// include the name of the parameter which was updated as well as
    /// its new value.
    pub fn make_parameter_handler(
        &mut self,
    ) -> Result<(
        impl Future<Output = ()> + Send,
        impl Stream<Item = (String, ParameterValue)>,
    )> {
        let mut handlers: Vec<std::pin::Pin<Box<dyn Future<Output = ()> + Send>>> = Vec::new();
        let (mut event_tx, event_rx) = mpsc::channel::<(String, ParameterValue)>(10);

        let node_name = self.name()?;
        let set_params_request_stream = self
            .create_service::<rcl_interfaces::srv::SetParameters::Service>(&format!(
                "{}/set_parameters",
                node_name
            ))?;

        let params = self.params.clone();
        let set_params_future = set_params_request_stream.for_each(
            move |req: ServiceRequest<rcl_interfaces::srv::SetParameters::Service>| {
                let mut result = rcl_interfaces::srv::SetParameters::Response::default();
                for p in &req.message.parameters {
                    let val = ParameterValue::from_parameter_value_msg(p.value.clone());
                    let changed = params
                        .lock()
                        .unwrap()
                        .get(&p.name)
                        .map(|v| v != &val)
                        .unwrap_or(true); // changed=true if new
                    params.lock().unwrap().insert(p.name.clone(), val.clone());
                    let r = rcl_interfaces::msg::SetParametersResult {
                        successful: true,
                        reason: "".into(),
                    };
                    result.results.push(r);
                    // if the value changed, send out new value on parameter event stream
                    if changed {
                        if let Err(e) = event_tx.try_send((p.name.clone(), val)) {
                            println!("Warning: could not send parameter event ({}).", e);
                        }
                    }
                }
                req.respond(result)
                    .expect("could not send reply to set parameter request");
                future::ready(())
            },
        );
        handlers.push(Box::pin(set_params_future));

        // rcl_interfaces/srv/GetParameters
        let get_params_request_stream = self
            .create_service::<rcl_interfaces::srv::GetParameters::Service>(&format!(
                "{}/get_parameters",
                node_name
            ))?;

        let params = self.params.clone();
        let get_params_future = get_params_request_stream.for_each(
            move |req: ServiceRequest<rcl_interfaces::srv::GetParameters::Service>| {
                let params = params.lock().unwrap();
                let values = req
                    .message
                    .names
                    .iter()
                    .map(|n| match params.get(n) {
                        Some(v) => v.clone(),
                        None => ParameterValue::NotSet,
                    })
                    .map(|v| v.into_parameter_value_msg())
                    .collect::<Vec<rcl_interfaces::msg::ParameterValue>>();

                let result = rcl_interfaces::srv::GetParameters::Response { values };
                req.respond(result)
                    .expect("could not send reply to set parameter request");
                future::ready(())
            },
        );

        handlers.push(Box::pin(get_params_future));

        // we don't care about the result, the futures will not complete anyway.
        Ok((join_all(handlers).map(|_| ()), event_rx))
    }

    /// Subscribe to a ROS topic.
    ///
    /// This function returns a `Stream` of ros messages.
    pub fn subscribe<T: 'static>(
        &mut self,
        topic: &str,
        qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = T> + Unpin>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle =
            create_subscription_helper(self.node_handle.as_mut(), topic, T::get_ts(), qos_profile)?;
        let (sender, receiver) = mpsc::channel::<T>(10);

        let ws = TypedSubscriber {
            rcl_handle: subscription_handle,
            sender,
        };
        self.subscribers.push(Box::new(ws));
        Ok(receiver)
    }

    /// Subscribe to a ROS topic.
    ///
    /// This function returns a `Stream` of ros messages without the rust convenience types.
    pub fn subscribe_native<T: 'static>(
        &mut self,
        topic: &str,
        qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = WrappedNativeMsg<T>> + Unpin>
    where
        T: WrappedTypesupport,
    {
        let subscription_handle =
            create_subscription_helper(self.node_handle.as_mut(), topic, T::get_ts(), qos_profile)?;
        let (sender, receiver) = mpsc::channel::<WrappedNativeMsg<T>>(10);

        let ws = NativeSubscriber {
            rcl_handle: subscription_handle,
            sender,
        };
        self.subscribers.push(Box::new(ws));
        Ok(receiver)
    }

    /// Subscribe to a ROS topic.
    ///
    /// This function returns a `Stream` of ros messages as `serde_json::Value`:s.
    /// Useful when you cannot know the type of the message at compile time.
    pub fn subscribe_untyped(
        &mut self,
        topic: &str,
        topic_type: &str,
        qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = Result<serde_json::Value>> + Unpin> {
        let msg = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let subscription_handle =
            create_subscription_helper(self.node_handle.as_mut(), topic, msg.ts, qos_profile)?;
        let (sender, receiver) = mpsc::channel::<Result<serde_json::Value>>(10);

        let ws = UntypedSubscriber {
            rcl_handle: subscription_handle,
            topic_type: topic_type.to_string(),
            sender,
        };
        self.subscribers.push(Box::new(ws));
        Ok(receiver)
    }

    /// Create a ROS service.
    ///
    /// This function returns a `Stream` of `ServiceRequest`:s. Call
    /// `respond` on the Service Request to send the reply.
    pub fn create_service<T: 'static>(
        &mut self,
        service_name: &str,
    ) -> Result<impl Stream<Item = ServiceRequest<T>> + Unpin>
    where
        T: WrappedServiceTypeSupport,
    {
        let service_handle =
            create_service_helper(self.node_handle.as_mut(), service_name, T::get_ts())?;
        let (sender, receiver) = mpsc::channel::<ServiceRequest<T>>(10);

        let ws = TypedService::<T> {
            rcl_handle: service_handle,
            outstanding_requests: vec![],
            sender,
        };

        self.services.push(Arc::new(Mutex::new(ws)));
        Ok(receiver)
    }

    /// Create a ROS service client.
    ///
    /// A service client is used to make requests to a ROS service server.
    pub fn create_client<T: 'static>(&mut self, service_name: &str) -> Result<Client<T>>
    where
        T: WrappedServiceTypeSupport,
    {
        let client_handle =
            create_client_helper(self.node_handle.as_mut(), service_name, T::get_ts())?;
        let ws = TypedClient::<T> {
            rcl_handle: client_handle,
            response_channels: Vec::new(),
            poll_available_channels: Vec::new(),
        };

        let client_arc = Arc::new(Mutex::new(ws));
        let c = make_client(Arc::downgrade(&client_arc));
        self.clients.push(client_arc);
        Ok(c)
    }

    /// Create a ROS service client.
    ///
    /// A service client is used to make requests to a ROS service
    /// server. This function returns an `UntypedClient`, which deals
    /// with `serde_json::Value`s instead of concrete types.  Useful
    /// when you cannot know the type of the message at compile time.
    pub fn create_client_untyped(
        &mut self,
        service_name: &str,
        service_type: &str,
    ) -> Result<ClientUntyped> {
        let service_type = UntypedServiceSupport::new_from(service_type)?;
        let client_handle =
            create_client_helper(self.node_handle.as_mut(), service_name, service_type.ts)?;
        let client = UntypedClient_ {
            service_type,
            rcl_handle: client_handle,
            response_channels: Vec::new(),
            poll_available_channels: Vec::new(),
        };

        let client_arc = Arc::new(Mutex::new(client));
        let c = make_untyped_client(Arc::downgrade(&client_arc));
        self.clients.push(client_arc);
        Ok(c)
    }

    /// Register a client for wakeup when the service or action server is available to the node.
    ///
    /// Returns a `Future` that completes when the service/action server is available.
    ///
    /// This function will register the client to be polled in
    /// `spin_once` until available, so spin_once must be called
    /// repeatedly in order to get the wakeup.
    pub fn is_available(
        &mut self,
        client: &dyn IsAvailablePollable,
    ) -> Result<impl Future<Output = Result<()>>> {
        let (sender, receiver) = oneshot::channel();
        client.register_poll_available(sender)?;
        Ok(receiver.map_err(|_| Error::RCL_RET_CLIENT_INVALID))
    }

    /// Create a ROS action client.
    ///
    /// An action client is used to make requests to a ROS action server.
    pub fn create_action_client<T: 'static>(&mut self, action_name: &str) -> Result<ActionClient<T>>
    where
        T: WrappedActionTypeSupport,
    {
        let client_handle =
            create_action_client_helper(self.node_handle.as_mut(), action_name, T::get_ts())?;
        let client = WrappedActionClient::<T> {
            rcl_handle: client_handle,
            goal_response_channels: Vec::new(),
            cancel_response_channels: Vec::new(),
            feedback_senders: Vec::new(),
            result_senders: Vec::new(),
            result_requests: Vec::new(),
            goal_status: HashMap::new(),
            poll_available_channels: Vec::new(),
        };

        let client_arc = Arc::new(Mutex::new(client));
        self.action_clients.push(client_arc.clone());
        let c = make_action_client(Arc::downgrade(&client_arc));
        Ok(c)
    }

    /// Create a ROS action client.
    ///
    /// A action client is used to make requests to a ROS service
    /// server. This function returns a `ActionClientUntyped`, which deals
    /// with `serde_json::Value`s instead of concrete types.  Useful
    /// when you cannot know the type of the message at compile time.
    pub fn create_action_client_untyped(
        &mut self,
        action_name: &str,
        action_type: &str,
    ) -> Result<ActionClientUntyped> {
        let action_type_support = UntypedActionSupport::new_from(action_type)?;
        let client_handle = create_action_client_helper(
            self.node_handle.as_mut(),
            action_name,
            action_type_support.ts,
        )?;
        let client = WrappedActionClientUntyped {
            action_type_support,
            rcl_handle: client_handle,
            goal_response_channels: Vec::new(),
            cancel_response_channels: Vec::new(),
            feedback_senders: Vec::new(),
            result_senders: Vec::new(),
            result_requests: Vec::new(),
            goal_status: HashMap::new(),
            poll_available_channels: Vec::new(),
        };

        let client_arc = Arc::new(Mutex::new(client));
        self.action_clients.push(client_arc.clone());
        let c = make_action_client_untyped(Arc::downgrade(&client_arc));
        Ok(c)
    }

    /// Create a ROS action server.
    ///
    /// This function returns a stream of `GoalRequest`s, which needs
    /// to be either accepted or rejected.
    pub fn create_action_server<T: 'static>(
        &mut self,
        action_name: &str,
    ) -> Result<impl Stream<Item = ActionServerGoalRequest<T>> + Unpin>
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

        let (goal_request_sender, goal_request_receiver) =
            mpsc::channel::<ActionServerGoalRequest<T>>(10);

        let server_handle = create_action_server_helper(
            self.node_handle.as_mut(),
            action_name,
            clock_handle.as_mut(),
            T::get_ts(),
        )?;
        let server = WrappedActionServer::<T> {
            rcl_handle: server_handle,
            clock_handle,
            goal_request_sender,
            active_cancel_requests: Vec::new(),
            cancel_senders: HashMap::new(),
            goals: HashMap::new(),
            result_msgs: HashMap::new(),
            result_requests: HashMap::new(),
        };

        let server_arc = Arc::new(Mutex::new(server));
        self.action_servers.push(server_arc);
        Ok(goal_request_receiver)
    }

    /// Create a ROS publisher.
    pub fn create_publisher<T>(
        &mut self,
        topic: &str,
        qos_profile: QosProfile,
    ) -> Result<Publisher<T>>
    where
        T: WrappedTypesupport,
    {
        let publisher_handle =
            create_publisher_helper(self.node_handle.as_mut(), topic, T::get_ts(), qos_profile)?;
        let arc = Arc::new(publisher_handle);
        let p = make_publisher(Arc::downgrade(&arc));
        self.pubs.push(arc);
        Ok(p)
    }

    /// Create a ROS publisher with a type given at runtime.
    pub fn create_publisher_untyped(
        &mut self,
        topic: &str,
        topic_type: &str,
        qos_profile: QosProfile,
    ) -> Result<PublisherUntyped> {
        let dummy = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let publisher_handle =
            create_publisher_helper(self.node_handle.as_mut(), topic, dummy.ts, qos_profile)?;
        let arc = Arc::new(publisher_handle);
        let p = make_publisher_untyped(Arc::downgrade(&arc), topic_type.to_owned());
        self.pubs.push(arc);
        Ok(p)
    }

    /// Spin the ROS node.
    ///
    /// This handles wakeups of all subscribes, services, etc on the
    /// ros side. In turn, this will complete future and wake up
    /// streams on the rust side. This needs to be called repeatedly
    /// (see the examples).
    ///
    /// `timeout` is a duration specifying how long the spin should
    /// block for if there are no pending events.
    pub fn spin_once(&mut self, timeout: Duration) {
        // first handle any completed action cancellation responses
        for a in &mut self.action_servers {
            a.lock().unwrap().send_completed_cancel_requests();
        }

        // as well as polling any services/action servers for availability
        for c in &mut self.clients {
            c.lock().unwrap().poll_available(self.node_handle.as_mut());
        }

        for c in &mut self.action_clients {
            c.lock().unwrap().poll_available(self.node_handle.as_mut());
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

            action_client_get_num_waits(
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

            action_server_get_num_waits(
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
                    self.subscribers.len() + total_action_subs,
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

        for s in &self.subscribers {
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
                rcl_wait_set_add_service(&mut ws, s.lock().unwrap().handle(), std::ptr::null_mut());
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
                rcl_action_wait_set_add_action_server(
                    &mut ws,
                    acs.lock().unwrap().handle(),
                    std::ptr::null_mut(),
                );
            }
        }

        let ret = unsafe { rcl_wait(&mut ws, timeout) };

        if ret == RCL_RET_TIMEOUT as i32 {
            unsafe {
                rcl_wait_set_fini(&mut ws);
            }
            return;
        }

        let ws_subs =
            unsafe { std::slice::from_raw_parts(ws.subscriptions, self.subscribers.len()) };
        let mut subs_to_remove = vec![];
        for (s, ws_s) in self.subscribers.iter_mut().zip(ws_subs) {
            if ws_s != &std::ptr::null() {
                let dropped = s.handle_incoming();
                if dropped {
                    s.destroy(&mut self.node_handle);
                    subs_to_remove.push(*s.handle());
                }
            }
        }
        self.subscribers
            .retain(|s| !subs_to_remove.contains(s.handle()));

        let ws_timers = unsafe { std::slice::from_raw_parts(ws.timers, self.timers.len()) };
        let mut timers_to_remove = vec![];
        for (s, ws_s) in self.timers.iter_mut().zip(ws_timers) {
            if ws_s != &std::ptr::null() {
                // TODO: move this to impl Timer
                let dropped = s.handle_incoming();
                if dropped {
                    timers_to_remove.push(s.timer_handle);
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
        let mut services_to_remove = vec![];
        for (s, ws_s) in self.services.iter_mut().zip(ws_services) {
            if ws_s != &std::ptr::null() {
                let mut service = s.lock().unwrap();
                let dropped = service.handle_request(s.clone());
                if dropped {
                    service.destroy(&mut self.node_handle);
                    services_to_remove.push(*service.handle());
                }
            }
        }
        self.services
            .retain(|s| !services_to_remove.contains(s.lock().unwrap().handle()));

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

    /// Returns a map of topic names and type names of the publishers
    /// visible to this node.
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

    /// Create a ROS wall timer.
    ///
    /// Create a ROS timer that is woken up by spin every `period`.
    pub fn create_wall_timer(&mut self, period: Duration) -> Result<Timer> {
        let mut clock = Clock::create(ClockType::SteadyTime)?;

        let mut timer_handle = unsafe { rcl_get_zero_initialized_timer() };

        let mut ctx = self.context.context_handle.lock().unwrap();
        let ret = unsafe {
            rcl_timer_init(
                &mut timer_handle,
                clock.clock_handle.as_mut(),
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

        let (tx, rx) = mpsc::channel::<Duration>(1);

        let timer = Timer_ {
            timer_handle,
            _clock: clock,
            sender: tx,
        };
        self.timers.push(timer);

        let out_timer = Timer { receiver: rx };

        Ok(out_timer)
    }

    /// Get the ros logger name for this node.
    pub fn logger(&self) -> &str {
        let ptr = unsafe { rcl_node_get_logger_name(self.node_handle.as_ref()) };
        if ptr.is_null() {
            return "";
        }
        let s = unsafe { CStr::from_ptr(ptr) };
        s.to_str().unwrap_or("")
    }
}

struct Timer_ {
    timer_handle: rcl_timer_t,
    _clock: Clock, // just here to be dropped properly later.
    sender: mpsc::Sender<Duration>,
}

impl Timer_ {
    fn handle_incoming(&mut self) -> bool {
        let mut is_ready = false;
        let ret = unsafe { rcl_timer_is_ready(&self.timer_handle, &mut is_ready) };
        if ret == RCL_RET_OK as i32 && is_ready {
            let mut nanos = 0i64;
            // todo: error handling
            let ret = unsafe { rcl_timer_get_time_since_last_call(&self.timer_handle, &mut nanos) };
            if ret == RCL_RET_OK as i32 {
                let ret = unsafe { rcl_timer_call(&mut self.timer_handle) };
                if ret == RCL_RET_OK as i32 {
                    if let Err(e) = self.sender.try_send(Duration::from_nanos(nanos as u64)) {
                        if e.is_disconnected() {
                            // client dropped the timer handle, let's drop our timer as well.
                            return true;
                        }
                        if e.is_full() {
                            println!(
                                "Warning: timer tick not handled in time - no wakeup will occur"
                            );
                        }
                    }
                }
            }
        }
        false
    }
}

impl Drop for Timer_ {
    fn drop(&mut self) {
        let _ret = unsafe { rcl_timer_fini(&mut self.timer_handle) };
    }
}

/// A ROS timer.
pub struct Timer {
    receiver: mpsc::Receiver<Duration>,
}

impl Timer {
    /// Completes when the next instant in the interval has been reached.
    ///
    /// Returns the time passed since the timer was last woken up.
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
fn wait_until_unwrapped<T>(mut a: Arc<T>) -> T {
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

        for s in &mut self.subscribers {
            s.destroy(&mut self.node_handle);
        }
        for s in &mut self.services {
            s.lock().unwrap().destroy(&mut self.node_handle);
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

pub trait IsAvailablePollable {
    fn register_poll_available(&self, sender: oneshot::Sender<()>) -> Result<()>;
}
