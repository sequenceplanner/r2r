use futures::{
    channel::{mpsc, oneshot},
    future::{self, join_all, FutureExt, TryFutureExt},
    stream::{Stream, StreamExt},
};
use indexmap::IndexMap;
use r2r_tracing::TracingId;
use std::{
    collections::HashMap,
    ffi::{CStr, CString},
    future::Future,
    marker::PhantomPinned,
    mem::MaybeUninit,
    pin::Pin,
    sync::{Arc, Mutex},
    time::Duration,
};

use r2r_actions::*;
use r2r_rcl::*;

#[cfg(r2r__rosgraph_msgs__msg__Clock)]
use crate::time_source::TimeSource;
use crate::{
    action_clients::*,
    action_clients_untyped::*,
    action_servers::*,
    clients::*,
    clocks::*,
    context::*,
    error::*,
    msg_types::{generated_msgs::rcl_interfaces, *},
    parameters::*,
    publishers::*,
    qos::QosProfile,
    services::*,
    subscribers::*,
};

/// A ROS Node.
///
/// This struct owns all subscribes, publishers, etc.  To get events
/// from the ROS network into your ros application, `spin_once` should
/// be called continously.
pub struct Node {
    context: Context,
    /// ROS parameters.
    pub params: Arc<Mutex<IndexMap<String, Parameter>>>,
    pub(crate) node_handle: Box<rcl_node_t>,
    // the node owns the subscribers
    pub(crate) subscribers: Vec<Box<dyn Subscriber_>>,
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
    pubs: Vec<Arc<Publisher_>>,
    // RosTime clock used by all timers created by create_timer()
    ros_clock: Arc<Mutex<Clock>>,
    // time source that provides simulated time
    #[cfg(r2r__rosgraph_msgs__msg__Clock)]
    time_source: TimeSource,
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
            log::error!("could not read parameters: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        if params.is_null() {
            return Ok(());
        }

        unsafe {
            if (*(*params.as_ref())).node_names == std::ptr::null_mut()
                || (*(*params.as_ref())).params == std::ptr::null_mut()
            {
                rcl_yaml_node_struct_fini(*params);
                return Ok(());
            }
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
            let mut params = self.params.lock().unwrap();
            if np.parameter_names != std::ptr::null_mut()
                && np.parameter_values != std::ptr::null_mut()
            {
                let param_names =
                    unsafe { std::slice::from_raw_parts(np.parameter_names, np.num_params) };

                let param_values =
                    unsafe { std::slice::from_raw_parts(np.parameter_values, np.num_params) };

                for (s, v) in param_names.iter().zip(param_values) {
                    let s = unsafe { CStr::from_ptr(*s) };
                    let key = s.to_str().unwrap_or("");
                    let val = ParameterValue::from_rcl(v);
                    params.insert(key.to_owned(), Parameter::new(val));
                }
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
            let ros_clock = Arc::new(Mutex::new(Clock::create(ClockType::RosTime)?));
            #[cfg(r2r__rosgraph_msgs__msg__Clock)]
            let time_source = {
                let time_source = TimeSource::new();
                time_source.attach_ros_clock(Arc::downgrade(&ros_clock))?;
                time_source
            };

            let mut node = Node {
                params: Arc::new(Mutex::new(IndexMap::new())),
                context: ctx,
                node_handle,
                subscribers: Vec::new(),
                services: Vec::new(),
                clients: Vec::new(),
                action_clients: Vec::new(),
                action_servers: Vec::new(),
                timers: Vec::new(),
                pubs: Vec::new(),
                ros_clock,
                #[cfg(r2r__rosgraph_msgs__msg__Clock)]
                time_source,
            };
            node.load_params()?;
            Ok(node)
        } else {
            log::error!("could not create node{}", res);
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
    ) -> Result<(impl Future<Output = ()> + Send, impl Stream<Item = (String, ParameterValue)>)>
    {
        self.make_parameter_handler_internal(None)
    }

    /// Creates parameter service handlers for the Node based on the
    /// [`RosParams`] trait.
    ///
    /// Supported parameter names and types are given by the
    /// `params_struct` parameter (usually referring to a structure).
    /// Fields of the structure will be updated based on the command
    /// line parameters (if any) and later whenever a parameter gets
    /// changed from external sources. Updated fields will be visible
    /// outside of the node via the GetParameters service.
    ///
    /// This function returns a tuple (`Future`, `Stream`), where the
    /// future should be spawned on onto the executor of choice. The
    /// `Stream` produces events whenever parameters change from
    /// external sources. The event elements of the event stream
    /// include the name of the parameter which was updated as well as
    /// its new value.
    pub fn make_derived_parameter_handler(
        &mut self, params_struct: Arc<Mutex<dyn RosParams + Send>>,
    ) -> Result<(impl Future<Output = ()> + Send, impl Stream<Item = (String, ParameterValue)>)>
    {
        self.make_parameter_handler_internal(Some(params_struct))
    }

    fn make_parameter_handler_internal(
        &mut self, params_struct: Option<Arc<Mutex<dyn RosParams + Send>>>,
    ) -> Result<(impl Future<Output = ()> + Send, impl Stream<Item = (String, ParameterValue)>)>
    {
        if let Some(ps) = &params_struct {
            // register all parameters
            ps.lock()
                .unwrap()
                .register_parameters("", None, &mut self.params.lock().unwrap())?;
        }
        let mut handlers: Vec<std::pin::Pin<Box<dyn Future<Output = ()> + Send>>> = Vec::new();
        let (mut event_tx, event_rx) = mpsc::channel::<(String, ParameterValue)>(10);

        let node_name = self.name()?;
        let params = self.params.clone();
        let params_struct_clone = params_struct.clone();
        let set_params_future = self
            .create_service_traced::<rcl_interfaces::srv::SetParameters::Service, _>(
                &format!("{}/set_parameters", node_name),
                QosProfile::default(),
                move |req: ServiceRequest<rcl_interfaces::srv::SetParameters::Service>| {
                    let mut result = rcl_interfaces::srv::SetParameters::Response::default();
                    for p in &req.message.parameters {
                        let val = ParameterValue::from_parameter_value_msg(p.value.clone());
                        let changed = params
                            .lock()
                            .unwrap()
                            .get(&p.name)
                            .map(|v| v.value != val)
                            .unwrap_or(true); // changed=true if new
                        let r = if let Some(ps) = &params_struct_clone {
                            // Update parameter structure
                            let result = ps.lock().unwrap().set_parameter(&p.name, &val);
                            if result.is_ok() {
                                // Also update Node::params
                                params
                                    .lock()
                                    .unwrap()
                                    .entry(p.name.clone())
                                    .and_modify(|p| p.value = val.clone());
                            }
                            rcl_interfaces::msg::SetParametersResult {
                                successful: result.is_ok(),
                                reason: result.err().map_or("".into(), |e| e.to_string()),
                            }
                        } else {
                            // No parameter structure - update only Node::params
                            params
                                .lock()
                                .unwrap()
                                .entry(p.name.clone())
                                .and_modify(|p| p.value = val.clone())
                                .or_insert(Parameter::new(val.clone()));
                            rcl_interfaces::msg::SetParametersResult {
                                successful: true,
                                reason: "".into(),
                            }
                        };
                        // if the value changed, send out new value on parameter event stream
                        if changed && r.successful {
                            if let Err(e) = event_tx.try_send((p.name.clone(), val)) {
                                log::debug!("Warning: could not send parameter event ({}).", e);
                            }
                        }
                        result.results.push(r);
                    }
                    req.respond(result)
                        .expect("could not send reply to set parameter request");
                },
            )?;
        handlers.push(Box::pin(set_params_future));

        // rcl_interfaces/srv/GetParameters
        let params = self.params.clone();
        let params_struct_clone = params_struct.clone();
        let get_params_future = self
            .create_service_traced::<rcl_interfaces::srv::GetParameters::Service, _>(
                &format!("{}/get_parameters", node_name),
                QosProfile::default(),
                move |req: ServiceRequest<rcl_interfaces::srv::GetParameters::Service>| {
                    let params = params.lock().unwrap();
                    let values = req
                        .message
                        .names
                        .iter()
                        .map(|n| {
                            // First try to get the parameter from the param structure
                            if let Some(ps) = &params_struct_clone {
                                if let Ok(value) = ps.lock().unwrap().get_parameter(n) {
                                    return value;
                                }
                            }
                            // Otherwise get it from node HashMap
                            match params.get(n) {
                                Some(v) => v.value.clone(),
                                None => ParameterValue::NotSet,
                            }
                        })
                        .map(|v| v.into_parameter_value_msg())
                        .collect::<Vec<rcl_interfaces::msg::ParameterValue>>();

                    let result = rcl_interfaces::srv::GetParameters::Response { values };
                    req.respond(result)
                        .expect("could not send reply to set parameter request");
                },
            )?;

        handlers.push(Box::pin(get_params_future));

        let params = self.params.clone();

        // rcl_interfaces/srv/ListParameters
        use rcl_interfaces::srv::ListParameters;
        let list_params_future = self.create_service_traced::<ListParameters::Service, _>(
            &format!("{}/list_parameters", node_name),
            QosProfile::default(),
            move |req: ServiceRequest<ListParameters::Service>| {
                Self::handle_list_parameters(req, &params)
            },
        )?;

        handlers.push(Box::pin(list_params_future));

        // rcl_interfaces/srv/DescribeParameters
        use rcl_interfaces::srv::DescribeParameters;
        let params = self.params.clone();
        let desc_params_future = self.create_service_traced::<DescribeParameters::Service, _>(
            &format!("{node_name}/describe_parameters"),
            QosProfile::default(),
            move |req: ServiceRequest<DescribeParameters::Service>| {
                Self::handle_desc_parameters(req, &params)
            },
        )?;

        handlers.push(Box::pin(desc_params_future));

        // rcl_interfaces/srv/GetParameterTypes
        use rcl_interfaces::srv::GetParameterTypes;
        let params = self.params.clone();
        let get_param_types_future = self.create_service_traced::<GetParameterTypes::Service, _>(
            &format!("{node_name}/get_parameter_types"),
            QosProfile::default(),
            move |req: ServiceRequest<GetParameterTypes::Service>| {
                let params = params.lock().unwrap();
                let types = req
                    .message
                    .names
                    .iter()
                    .map(|name| match params.get(name) {
                        Some(param) => param.value.into_parameter_type(),
                        None => rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET as u8,
                    })
                    .collect();
                req.respond(GetParameterTypes::Response { types })
                    .expect("could not send reply to get parameter types request");
            },
        )?;

        handlers.push(Box::pin(get_param_types_future));

        #[cfg(r2r__rosgraph_msgs__msg__Clock)]
        {
            // create TimeSource based on value of use_sim_time parameter
            let use_sim_time = {
                let params = self.params.lock().unwrap();
                params
                    .get("use_sim_time")
                    .and_then(|param| {
                        if let ParameterValue::Bool(val) = param.value {
                            Some(val)
                        } else {
                            log::error!("Parameter use_sim_time is not bool. Assuming false");
                            None
                        }
                    })
                    .unwrap_or(false)
            };
            if use_sim_time {
                let ts = self.time_source.clone();
                ts.enable_sim_time(self)?;
            }
        }

        // we don't care about the result, the futures will not complete anyway.
        Ok((join_all(handlers).map(|_| ()), event_rx))
    }

    fn handle_list_parameters(
        req: ServiceRequest<rcl_interfaces::srv::ListParameters::Service>,
        params: &Arc<Mutex<IndexMap<String, Parameter>>>,
    ) {
        use rcl_interfaces::srv::ListParameters;

        let depth = req.message.depth;
        let prefixes = &req.message.prefixes;
        let separator = '.';
        let params = params.lock().unwrap();
        let mut result = rcl_interfaces::msg::ListParametersResult {
            names: vec![],
            prefixes: vec![],
        };
        for (name, _) in params.iter().filter(|(name, _)| {
            let get_all = prefixes.is_empty()
                && ((depth == ListParameters::Request::DEPTH_RECURSIVE as u64)
                    || name.matches(separator).count() < depth as usize);
            let prefix_matches = prefixes.iter().any(|prefix| {
                if *name == prefix {
                    return true;
                } else if name.starts_with(&format!("{prefix}{separator}")) {
                    let substr = &name[prefix.len()..];
                    return (depth == ListParameters::Request::DEPTH_RECURSIVE as u64)
                        || substr.matches(separator).count() < depth as usize;
                }
                false
            });
            get_all || prefix_matches
        }) {
            result.names.push(name.clone());
            if let Some(last_separator) = name.rfind(separator) {
                let prefix = &name[0..last_separator];
                if result.prefixes.iter().all(|p| p != prefix) {
                    result.prefixes.push(prefix.to_string());
                }
            }
        }
        req.respond(ListParameters::Response { result })
            .expect("could not send reply to list parameter request");
    }

    fn handle_desc_parameters(
        req: ServiceRequest<rcl_interfaces::srv::DescribeParameters::Service>,
        params: &Arc<Mutex<IndexMap<String, Parameter>>>,
    ) {
        use rcl_interfaces::{msg::ParameterDescriptor, srv::DescribeParameters};
        let mut descriptors = Vec::<ParameterDescriptor>::new();
        let params = params.lock().unwrap();
        for name in &req.message.names {
            let default = Parameter::empty();
            let param = params.get(name).unwrap_or(&default);
            descriptors.push(ParameterDescriptor {
                name: name.clone(),
                type_: param.value.into_parameter_type(),
                description: param.description.to_string(),
                ..Default::default()
            });
        }
        req.respond(DescribeParameters::Response { descriptors })
            .expect("could not send reply to describe parameters request");
    }

    /// Fetch a single ROS parameter.
    pub fn get_parameter<T>(&self, name: &str) -> Result<T>
    where
        ParameterValue: TryInto<T, Error = WrongParameterType>,
    {
        let params = self.params.lock().unwrap();
        let value = params
            .get(name)
            .map(|parameter| parameter.value.clone())
            .unwrap_or(ParameterValue::NotSet);

        let value: T =
            value
                .try_into()
                .map_err(|error: WrongParameterType| Error::ParameterWrongType {
                    name: name.to_string(),
                    expected_type: error.expected_type_name,
                    actual_type: error.actual_type_name,
                })?;

        Ok(value)
    }

    /// Subscribe to a ROS topic.
    ///
    /// This function returns a `Stream` of ros messages.
    pub fn subscribe<T: 'static>(
        &mut self, topic: &str, qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = T> + Unpin>
    where
        T: WrappedTypesupport,
    {
        let (sender, receiver) = mpsc::channel::<T>(10);

        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut subscription = Box::new(TypedSubscriber {
            rcl_handle: unsafe { rcl_get_zero_initialized_subscription() },
            sender,
        });

        // SAFETY:
        // create_subscription_helper requires zero initialized subscription_handle -> done above
        // Completes initialization of subscription.
        unsafe {
            create_subscription_helper(
                &mut subscription.rcl_handle,
                self.node_handle.as_mut(),
                topic,
                T::get_ts(),
                qos_profile,
            )?;
        };

        r2r_tracing::trace_subscription_init(&subscription.rcl_handle, &*subscription);

        self.subscribers.push(subscription);
        Ok(receiver)
    }

    /// Subscribe to a ROS topic and create tracepoints around the callback.
    ///
    /// This function takes a `callback`, and associates it with the
    /// subscription, producing a `Future` representing this task.
    /// The callback will be called with received messages.
    ///
    /// Each time the callback is called, its start and end time will be traced by the
    /// ROS 2 tracing framework.
    ///
    /// You must spawn or await the returned [`Future`] otherwise the callback
    /// will never be called.
    pub fn subscribe_traced<T, F>(
        &mut self, topic: &str, qos_profile: QosProfile, callback: F,
    ) -> Result<impl Future<Output = ()> + Unpin>
    where
        T: WrappedTypesupport + 'static,
        F: FnMut(T),
    {
        let (sender, receiver) = mpsc::channel::<T>(10);

        // SAFETY: The `rcl_handle` is not fully initialized yet.
        let mut subscription = Box::new(TypedSubscriber {
            rcl_handle: unsafe { rcl_get_zero_initialized_subscription() },
            sender,
        });

        // SAFETY:
        // create_subscription_helper requires zero initialized subscription_handle -> done above
        // The `rcl_handle` is fully initialized here in `create_subscription_helper`.
        unsafe {
            create_subscription_helper(
                &mut subscription.rcl_handle,
                self.node_handle.as_mut(),
                topic,
                T::get_ts(),
                qos_profile,
            )?;
        };

        r2r_tracing::trace_subscription_init(&subscription.rcl_handle, &*subscription);

        let mut callback = r2r_tracing::Callback::new_subscription(&*subscription, callback);
        let fut = receiver.for_each(move |msg| {
            callback.call(msg);
            future::ready(())
        });

        self.subscribers.push(subscription);
        Ok(fut)
    }

    /// Subscribe to a ROS topic.
    ///
    /// This function returns a `Stream` of ros messages without the rust convenience types.
    pub fn subscribe_native<T: 'static>(
        &mut self, topic: &str, qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = WrappedNativeMsg<T>> + Unpin>
    where
        T: WrappedTypesupport,
    {
        let (sender, receiver) = mpsc::channel::<WrappedNativeMsg<T>>(10);

        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut subscription = Box::new(NativeSubscriber {
            rcl_handle: unsafe { rcl_get_zero_initialized_subscription() },
            sender,
        });

        // SAFETY:
        // create_subscription_helper requires zero initialized subscription_handle -> done above
        // Completes initialization of subscription.
        unsafe {
            create_subscription_helper(
                &mut subscription.rcl_handle,
                self.node_handle.as_mut(),
                topic,
                T::get_ts(),
                qos_profile,
            )?;
        };

        r2r_tracing::trace_subscription_init(&subscription.rcl_handle, &*subscription);

        self.subscribers.push(subscription);
        Ok(receiver)
    }

    /// Subscribe to a ROS topic.
    ///
    /// This function returns a `Stream` of ros messages as `serde_json::Value`:s.
    /// Useful when you cannot know the type of the message at compile time.
    pub fn subscribe_untyped(
        &mut self, topic: &str, topic_type: &str, qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = Result<serde_json::Value>> + Unpin> {
        let msg = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let (sender, receiver) = mpsc::channel::<Result<serde_json::Value>>(10);

        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut subscription = Box::new(UntypedSubscriber {
            rcl_handle: unsafe { rcl_get_zero_initialized_subscription() },
            topic_type: topic_type.to_string(),
            sender,
        });

        // SAFETY:
        // create_subscription_helper requires zero initialized subscription_handle -> done above
        // Completes initialization of subscription.
        unsafe {
            create_subscription_helper(
                &mut subscription.rcl_handle,
                self.node_handle.as_mut(),
                topic,
                msg.ts,
                qos_profile,
            )?;
        };

        r2r_tracing::trace_subscription_init(&subscription.rcl_handle, &*subscription);

        self.subscribers.push(subscription);
        Ok(receiver)
    }

    /// Subscribe to a ROS topic.
    ///
    /// This function returns a `Stream` of ros messages as non-deserialized `Vec<u8>`:s.
    /// Useful if you just want to pass the data along to another part of the system.
    pub fn subscribe_raw(
        &mut self, topic: &str, topic_type: &str, qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = Vec<u8>> + Unpin> {
        // TODO is it possible to handle the raw message without type support?
        //
        // Passing null ts to rcl_subscription_init throws an error ..
        //
        // It does not seem possible to not have a type support, which is a shame
        // because it means we always have to build the message types even if we
        // are just after the raw bytes.
        let msg = WrappedNativeMsgUntyped::new_from(topic_type)?;

        // Keep a buffer to reduce number of allocations. The rmw will
        // resize it if the message size exceeds the buffer size.
        let mut msg_buf: rcl_serialized_message_t =
            unsafe { rcutils_get_zero_initialized_uint8_array() };
        let ret = unsafe {
            rcutils_uint8_array_init(
                &mut msg_buf as *mut rcl_serialized_message_t,
                0,
                &rcutils_get_default_allocator(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            return Err(Error::from_rcl_error(ret));
        }

        let (sender, receiver) = mpsc::channel::<Vec<u8>>(10);

        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut subscription = Box::new(RawSubscriber {
            rcl_handle: unsafe { rcl_get_zero_initialized_subscription() },
            msg_buf,
            sender,
        });

        // SAFETY:
        // create_subscription_helper requires zero initialized subscription_handle -> done above
        // Completes initialization of subscription.
        unsafe {
            create_subscription_helper(
                &mut subscription.rcl_handle,
                self.node_handle.as_mut(),
                topic,
                msg.ts,
                qos_profile,
            )?;
        };

        r2r_tracing::trace_subscription_init(&subscription.rcl_handle, &*subscription);

        self.subscribers.push(subscription);
        Ok(receiver)
    }

    /// Create a ROS service.
    ///
    /// This function returns a `Stream` of `ServiceRequest`:s. Call
    /// `respond` on the Service Request to send the reply.
    pub fn create_service<T: 'static>(
        &mut self, service_name: &str, qos_profile: QosProfile,
    ) -> Result<impl Stream<Item = ServiceRequest<T>> + Unpin>
    where
        T: WrappedServiceTypeSupport,
    {
        let (sender, receiver) = mpsc::channel::<ServiceRequest<T>>(10);

        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut service_arc = Arc::new(Mutex::new(TypedService::<T> {
            rcl_handle: unsafe { rcl_get_zero_initialized_service() },
            sender,
        }));
        let service_ref = Arc::get_mut(&mut service_arc)
            .unwrap() // No other Arc should exist. The Arc was just created.
            .get_mut()
            .unwrap(); // The mutex was just created. It should not be poisoned.

        // SAFETY:
        // The service was zero initialized above.
        // Full initialization happens in `create_service_helper``.
        unsafe {
            create_service_helper(
                &mut service_ref.rcl_handle,
                self.node_handle.as_mut(),
                service_name,
                T::get_ts(),
                qos_profile,
            )?;
        };

        // Only push after full initialization.
        self.services.push(service_arc);
        Ok(receiver)
    }

    /// Create a ROS service and create tracepoints around the callback.
    ///
    /// This function takes a `callback`, and associates it with the
    /// service, producing a `Future` representing this task.
    /// The callback will be called with received requests.
    ///
    /// Each time the callback is called, its start and end time will be traced by the
    /// ROS 2 tracing framework.
    ///
    /// You must spawn or await the returned [`Future`] otherwise the callback
    /// will never be called.
    pub fn create_service_traced<T, F>(
        &mut self, service_name: &str, qos_profile: QosProfile, callback: F,
    ) -> Result<impl Future<Output = ()> + Unpin>
    where
        T: WrappedServiceTypeSupport + 'static,
        F: FnMut(ServiceRequest<T>),
    {
        let (sender, receiver) = mpsc::channel::<ServiceRequest<T>>(10);

        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut service_arc = Arc::new(Mutex::new(TypedService::<T> {
            rcl_handle: unsafe { rcl_get_zero_initialized_service() },
            sender,
        }));

        let service_ref = Arc::get_mut(&mut service_arc)
            .unwrap() // No other Arc should exist. The Arc was just created.
            .get_mut()
            .unwrap(); // The mutex was just created. It should not be poisoned.

        // SAFETY:
        // The service was zero initialized above.
        // Full initialization happens in `create_service_helper`.
        unsafe {
            create_service_helper(
                &mut service_ref.rcl_handle,
                self.node_handle.as_mut(),
                service_name,
                T::get_ts(),
                qos_profile,
            )?;
        };

        let mut callback = r2r_tracing::Callback::new_service(&service_ref.rcl_handle, callback);
        let fut = receiver.for_each(move |req| {
            callback.call(req);
            future::ready(())
        });

        // Only push after full initialization.
        self.services.push(service_arc);
        Ok(fut)
    }

    /// Create a ROS service client.
    ///
    /// A service client is used to make requests to a ROS service server.
    pub fn create_client<T: 'static>(
        &mut self, service_name: &str, qos_profile: QosProfile,
    ) -> Result<Client<T>>
    where
        T: WrappedServiceTypeSupport,
    {
        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut client_arc = Arc::new(Mutex::new(TypedClient::<T> {
            rcl_handle: unsafe { rcl_get_zero_initialized_client() },
            response_channels: Vec::new(),
            poll_available_channels: Vec::new(),
        }));
        let client_ref = Arc::get_mut(&mut client_arc)
            .unwrap() // No other Arc should exist. The Arc was just created.
            .get_mut()
            .unwrap(); // The mutex was just created. It should not be poisoned.

        // SAFETY:
        // The client was zero initialized above.
        // Full initialization happens in `create_client_helper`.
        unsafe {
            create_client_helper(
                &mut client_ref.rcl_handle,
                self.node_handle.as_mut(),
                service_name,
                T::get_ts(),
                qos_profile,
            )?;
        };

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
        &mut self, service_name: &str, service_type: &str, qos_profile: QosProfile,
    ) -> Result<ClientUntyped> {
        let service_type = UntypedServiceSupport::new_from(service_type)?;

        // SAFETY: The `rcl_handle` is zero initialized (partial initialization) in this block.
        let mut client_arc = Arc::new(Mutex::new(UntypedClient_ {
            service_type,
            rcl_handle: unsafe { rcl_get_zero_initialized_client() },
            response_channels: Vec::new(),
            poll_available_channels: Vec::new(),
        }));
        let client_ref = Arc::get_mut(&mut client_arc)
            .unwrap() // No other Arc should exist. The Arc was just created.
            .get_mut()
            .unwrap(); // The mutex was just created. It should not be poisoned.

        // SAFETY:
        // The client was zero initialized above.
        // Full initialization happens in `create_client_helper`.
        unsafe {
            create_client_helper(
                &mut client_ref.rcl_handle,
                self.node_handle.as_mut(),
                service_name,
                client_ref.service_type.ts,
                qos_profile,
            )?;
        };

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
        &mut self, action_name: &str, action_type: &str,
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
        &mut self, action_name: &str,
    ) -> Result<impl Stream<Item = ActionServerGoalRequest<T>> + Unpin>
    where
        T: WrappedActionTypeSupport,
    {
        // for now automatically create a ros clock...
        let mut clock_handle = MaybeUninit::<rcl_clock_t>::uninit();
        let ret = unsafe {
            rcl_ros_clock_init(clock_handle.as_mut_ptr(), &mut rcutils_get_default_allocator())
        };
        if ret != RCL_RET_OK as i32 {
            log::error!("could not create steady clock: {}", ret);
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
        &mut self, topic: &str, qos_profile: QosProfile,
    ) -> Result<Publisher<T>>
    where
        T: WrappedTypesupport,
    {
        let publisher_arc =
            create_publisher_helper(self.node_handle.as_mut(), topic, T::get_ts(), qos_profile)?;
        let p = make_publisher(Arc::downgrade(&publisher_arc));
        self.pubs.push(publisher_arc);
        Ok(p)
    }

    /// Create a ROS publisher with a type given at runtime, where the data may either be
    /// supplied as JSON (using the `publish` method) or a pre-serialized ROS message
    /// (i.e. &[u8], using the `publish_raw` method).
    pub fn create_publisher_untyped(
        &mut self, topic: &str, topic_type: &str, qos_profile: QosProfile,
    ) -> Result<PublisherUntyped> {
        let dummy = WrappedNativeMsgUntyped::new_from(topic_type)?;
        let publisher_arc =
            create_publisher_helper(self.node_handle.as_mut(), topic, dummy.ts, qos_profile)?;
        let p = make_publisher_untyped(Arc::downgrade(&publisher_arc), topic_type.to_owned());
        self.pubs.push(publisher_arc);
        Ok(p)
    }

    /// Destroy a ROS publisher.
    pub fn destroy_publisher<T: WrappedTypesupport>(&mut self, p: Publisher<T>) {
        if let Some(handle) = p.handle.upgrade() {
            // Remove handle from list of publishers.
            self.pubs.iter().position(|p| Arc::ptr_eq(p, &handle))
                .map(|i| self.pubs.swap_remove(i));

            let handle = wait_until_unwrapped(handle);
            handle.destroy(self.node_handle.as_mut());
        }
    }

    /// Destroy a ROS publisher.
    pub fn destroy_publisher_untyped(&mut self, p: PublisherUntyped) {
        if let Some(handle) = p.handle.upgrade() {
            // Remove handle from list of publishers.
            self.pubs.iter().position(|p| Arc::ptr_eq(p, &handle))
                .map(|i| self.pubs.swap_remove(i));

            let handle = wait_until_unwrapped(handle);
            handle.destroy(self.node_handle.as_mut());
        }
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
        r2r_tracing::trace_spin_start(&*self.node_handle, timeout);

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

        for p in &self.pubs {
            p.poll_has_inter_process_subscribers();
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
                rcl_wait_set_add_timer(&mut ws, s.get_handle(), std::ptr::null_mut());
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
            r2r_tracing::trace_spin_timeout(&*self.node_handle);
            return;
        }

        r2r_tracing::trace_spin_wake(&*self.node_handle);

        let mut subs_to_remove = vec![];
        if ws.subscriptions != std::ptr::null_mut() {
            let ws_subs =
                unsafe { std::slice::from_raw_parts(ws.subscriptions, self.subscribers.len()) };
            for (s, ws_s) in self.subscribers.iter_mut().zip(ws_subs) {
                if ws_s != &std::ptr::null() {
                    let dropped = s.handle_incoming();
                    if dropped {
                        s.destroy(&mut self.node_handle);
                        subs_to_remove.push(*s.handle());
                    }
                }
            }
        }
        self.subscribers
            .retain(|s| !subs_to_remove.contains(s.handle()));

        let mut timers_to_remove = vec![];
        if ws.timers != std::ptr::null_mut() {
            let ws_timers = unsafe { std::slice::from_raw_parts(ws.timers, self.timers.len()) };
            for (s, ws_s) in self.timers.iter_mut().zip(ws_timers) {
                if ws_s != &std::ptr::null() {
                    // TODO: move this to impl Timer
                    let dropped = s.handle_incoming();
                    if dropped {
                        timers_to_remove.push((*s.timer_handle).to_owned());
                    }
                }
            }
        }
        // drop timers scheduled for deletion
        self.timers
            .retain(|t| !timers_to_remove.contains(&*t.timer_handle));

        if ws.clients != std::ptr::null_mut() {
            let ws_clients = unsafe { std::slice::from_raw_parts(ws.clients, self.clients.len()) };
            for (s, ws_s) in self.clients.iter_mut().zip(ws_clients) {
                if ws_s != &std::ptr::null() {
                    let mut s = s.lock().unwrap();
                    s.handle_response();
                }
            }
        }

        let mut services_to_remove = vec![];
        if ws.services != std::ptr::null_mut() {
            let ws_services =
                unsafe { std::slice::from_raw_parts(ws.services, self.services.len()) };
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

        r2r_tracing::trace_spin_end(&*self.node_handle);
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
            log::error!("could not get topic names and types {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        let mut res = HashMap::new();
        if tnat.names.data != std::ptr::null_mut() && tnat.types != std::ptr::null_mut() {
            let names = unsafe { std::slice::from_raw_parts(tnat.names.data, tnat.names.size) };
            let types = unsafe { std::slice::from_raw_parts(tnat.types, tnat.names.size) };

            for (n, t) in names.iter().zip(types) {
                assert!(!(*n).is_null());
                let topic_name = unsafe { CStr::from_ptr(*n).to_str().unwrap().to_owned() };
                assert!(!t.data.is_null());
                let topic_types = unsafe { std::slice::from_raw_parts(t, t.size) };
                let topic_types: Vec<String> = unsafe {
                    topic_types
                        .iter()
                        .map(|t| {
                            assert!(*(t.data) != std::ptr::null_mut());
                            CStr::from_ptr(*(t.data)).to_str().unwrap().to_owned()
                        })
                        .collect()
                };
                res.insert(topic_name, topic_types);
            }
        }
        unsafe {
            rmw_names_and_types_fini(&mut tnat);
        } // TODO: check return value
        Ok(res)
    }

    pub fn get_publishers_info_by_topic(
        &self, topic_name: &str, no_mangle: bool,
    ) -> Result<Vec<TopicEndpointInfo>> {
        let node = self.node_handle.as_ref();

        let topic_c_string =
            CString::new(topic_name).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

        let mut allocator = unsafe { rcutils_get_default_allocator() };

        let mut info_array: rcl_topic_endpoint_info_array_t =
            unsafe { rmw_get_zero_initialized_topic_endpoint_info_array() };

        let result = unsafe {
            rcl_get_publishers_info_by_topic(
                node,
                &mut allocator,
                topic_c_string.as_ptr(),
                no_mangle,
                &mut info_array,
            )
        };

        if result != RCL_RET_OK as i32 {
            unsafe { rmw_topic_endpoint_info_array_fini(&mut info_array, &mut allocator) };
            return Err(Error::from_rcl_error(result));
        }

        // Convert info_array to Vec<TopicEndpointInfo>
        let topic_info_list = convert_info_array_to_vec(&info_array);

        let result = unsafe { rmw_topic_endpoint_info_array_fini(&mut info_array, &mut allocator) };

        if result != RCL_RET_OK as i32 {
            return Err(Error::from_rcl_error(result));
        }

        Ok(topic_info_list)
    }

    /// Create a ROS wall timer.
    ///
    /// Create a ROS timer that is woken up by spin every `period`.
    ///
    /// This timer uses [`ClockType::SteadyTime`] clock.
    pub fn create_wall_timer(&mut self, period: Duration) -> Result<Timer> {
        let mut clock = Clock::create(ClockType::SteadyTime)?;
        let timer_handle = self.create_timer_helper(&mut clock, period)?;

        let (tx, rx) = mpsc::channel::<Duration>(1);

        let timer = Timer_ {
            timer_handle,
            _clock: Some(clock), // The timer owns the clock.
            sender: tx,
        };

        let out_timer = unsafe {
            Timer {
                receiver: rx,
                timer_handle: TracingId::new(timer.get_handle()),
                node_handle: TracingId::new(&*self.node_handle),
            }
        };
        self.timers.push(timer);

        Ok(out_timer)
    }

    /// Create a ROS timer
    ///
    /// Create a ROS timer that is woken up by spin every `period`.
    ///
    /// This timer uses node's [`ClockType::RosTime`] clock.
    pub fn create_timer(&mut self, period: Duration) -> Result<Timer> {
        let mut clock = self.ros_clock.lock().unwrap();
        let timer_handle = self.create_timer_helper(&mut clock, period)?;

        let (tx, rx) = mpsc::channel::<Duration>(1);

        let timer = Timer_ {
            timer_handle,
            _clock: None, // The timer does not own the clock (the node owns it).
            sender: tx,
        };

        let out_timer = unsafe {
            Timer {
                receiver: rx,
                timer_handle: TracingId::new(timer.get_handle()),
                node_handle: TracingId::new(&*self.node_handle),
            }
        };
        self.timers.push(timer);

        Ok(out_timer)
    }

    fn create_timer_helper(
        &self, clock: &mut Clock, period: Duration,
    ) -> Result<Pin<Box<RclTimer>>> {
        // rcl expects that the address of the rcl_timer_t does not change.
        let mut timer_handle = unsafe { Box::pin(RclTimer::new()) };

        let mut ctx = self.context.context_handle.lock().unwrap();
        let ret = unsafe {
            rcl_timer_init(
                &mut timer_handle.as_mut().get_unchecked_mut().handle,
                clock.clock_handle.as_mut(),
                ctx.as_mut(),
                period.as_nanos() as i64,
                None,
                rcutils_get_default_allocator(),
            )
        };

        if ret != RCL_RET_OK as i32 {
            log::error!("could not create timer: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        Ok(timer_handle)
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

    /// Get TimeSource of the node
    ///
    /// See: [`TimeSource`]
    #[cfg(r2r__rosgraph_msgs__msg__Clock)]
    pub fn get_time_source(&self) -> TimeSource {
        self.time_source.clone()
    }

    /// Get ROS clock of the node
    ///
    /// This is the same clock that is used by ROS timers created in [`Node::create_timer`].
    pub fn get_ros_clock(&self) -> Arc<Mutex<Clock>> {
        self.ros_clock.clone()
    }
}

#[derive(Debug, Clone, PartialEq)]
struct RclTimer {
    handle: rcl_timer_t,
    _pin: PhantomPinned, // To prevent Unpin implementation
}

impl RclTimer {
    unsafe fn new() -> Self {
        Self {
            handle: rcl_get_zero_initialized_timer(),
            _pin: PhantomPinned,
        }
    }
}

struct Timer_ {
    timer_handle: Pin<Box<RclTimer>>,
    _clock: Option<Clock>, // Some(clock) if the timer owns the clock, just here to be dropped properly later.
    sender: mpsc::Sender<Duration>,
}

impl Timer_ {
    fn get_handle(&self) -> *const rcl_timer_t {
        &self.timer_handle.handle
    }

    /// Get mutable pointer to handle
    ///
    /// SAFETY:
    ///     Pointer valid only for modification.
    ///     Must not invalidate or replace the timer unless in Drop.
    unsafe fn get_handle_mut(&mut self) -> *mut rcl_timer_t {
        self.timer_handle
            .as_mut()
            .map_unchecked_mut(|s| &mut s.handle)
            .get_unchecked_mut()
    }

    fn handle_incoming(&mut self) -> bool {
        let mut is_ready = false;
        let ret = unsafe { rcl_timer_is_ready(self.get_handle(), &mut is_ready) };
        if ret == RCL_RET_OK as i32 && is_ready {
            let mut nanos = 0i64;
            // todo: error handling
            let ret = unsafe { rcl_timer_get_time_since_last_call(self.get_handle(), &mut nanos) };
            if ret == RCL_RET_OK as i32 {
                let ret = unsafe { rcl_timer_call(self.get_handle_mut()) };
                if ret == RCL_RET_OK as i32 {
                    if let Err(e) = self.sender.try_send(Duration::from_nanos(nanos as u64)) {
                        if e.is_disconnected() {
                            // client dropped the timer handle, let's drop our timer as well.
                            return true;
                        }
                        if e.is_full() {
                            log::debug!(
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
        let _ret = unsafe { rcl_timer_fini(self.get_handle_mut()) };
    }
}

/// A ROS timer.
pub struct Timer {
    receiver: mpsc::Receiver<Duration>,
    timer_handle: TracingId<rcl_timer_t>,
    node_handle: TracingId<rcl_node_t>,
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

    /// Transforms this timer stream to a [`Future`] calling the given `callback` on each tick.
    ///
    /// The callback execution is traced by r2r_tracing.
    ///
    /// This function should be called before dropping the timer's node.
    /// Otherwise, the trace data might be inconsistent.
    #[must_use = "Futures do nothing unless you `.await` or poll them"]
    pub fn on_tick<F: FnMut(Duration)>(self, callback: F) -> impl Future<Output = ()> + Unpin {
        let mut callback = r2r_tracing::Callback::new_timer(self.timer_handle, callback);
        r2r_tracing::trace_timer_link_node(self.timer_handle, self.node_handle);

        self.receiver.for_each(move |duration| {
            callback.call(duration);
            future::ready(())
        })
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
        for c in &mut self.clients {
            c.lock().unwrap().destroy(&mut self.node_handle);
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
            let p = wait_until_unwrapped(p);

            p.destroy(self.node_handle.as_mut());
        }
        unsafe {
            rcl_node_fini(self.node_handle.as_mut());
        }
    }
}

pub trait IsAvailablePollable {
    fn register_poll_available(&self, sender: oneshot::Sender<()>) -> Result<()>;
}

pub struct TopicEndpointInfo {
    pub node_name: String,
    pub node_namespace: String,
    pub topic_type: String,
    pub endpoint_gid: [u8; RMW_GID_STORAGE_SIZE as usize],
    pub qos_profile: QosProfile,
}

impl From<rmw_topic_endpoint_info_t> for TopicEndpointInfo {
    fn from(info: rmw_topic_endpoint_info_t) -> Self {
        // Convert C strings to Rust String
        let node_name = unsafe { CStr::from_ptr(info.node_name) }
            .to_string_lossy()
            .into_owned();
        let node_namespace = unsafe { CStr::from_ptr(info.node_namespace) }
            .to_string_lossy()
            .into_owned();
        let topic_type = unsafe { CStr::from_ptr(info.topic_type) }
            .to_string_lossy()
            .into_owned();

        // Copy the endpoint_gid array
        let endpoint_gid: [u8; RMW_GID_STORAGE_SIZE as usize] = info.endpoint_gid;

        // Convert qos_profile
        let qos_profile = QosProfile::from(info.qos_profile); // Adjust this line based on how QosProfile is defined

        TopicEndpointInfo {
            node_name,
            node_namespace,
            topic_type,
            endpoint_gid,
            qos_profile,
        }
    }
}

fn convert_info_array_to_vec(
    info_array: &rcl_topic_endpoint_info_array_t,
) -> Vec<TopicEndpointInfo> {
    let mut topic_info_list = Vec::with_capacity(info_array.size);

    if info_array.info_array != std::ptr::null_mut() {
        unsafe {
            let infos = std::slice::from_raw_parts(info_array.info_array, info_array.size);
            for &info in infos {
                let endpoint_info = TopicEndpointInfo::from(info);
                topic_info_list.push(endpoint_info);
            }
        }
    }

    topic_info_list
}
