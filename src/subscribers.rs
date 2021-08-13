use super::*;

pub trait Subscriber {
    fn handle(&self) -> &rcl_subscription_t;
    fn handle_incoming(&mut self) -> ();
    fn destroy(&mut self, node: &mut rcl_node_t) -> ();
}

pub struct TypedSubscriber<T>
where
    T: WrappedTypesupport,
{
    pub rcl_handle: rcl_subscription_t,
    pub sender: mpsc::Sender<T>,
}

pub struct NativeSubscriber<T>
where
    T: WrappedTypesupport,
{
    pub rcl_handle: rcl_subscription_t,
    pub sender: mpsc::Sender<WrappedNativeMsg<T>>,
}

pub struct UntypedSubscriber {
    pub rcl_handle: rcl_subscription_t,
    pub topic_type: String,
    pub sender: mpsc::Sender<Result<serde_json::Value>>,
}

impl<T: 'static> Subscriber for TypedSubscriber<T>
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

impl<T: 'static> Subscriber for NativeSubscriber<T>
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

impl Subscriber for UntypedSubscriber {
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
