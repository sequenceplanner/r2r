use futures::channel::mpsc;
use std::ffi::CString;

use crate::error::*;
use crate::msg_types::*;
use crate::qos::QosProfile;
use r2r_rcl::*;

pub trait Subscriber_ {
    fn handle(&self) -> &rcl_subscription_t;
    /// Returns true if the subscriber stream has been dropped.
    fn handle_incoming(&mut self) -> bool;
    fn destroy(&mut self, node: &mut rcl_node_t);
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

impl<T: 'static> Subscriber_ for TypedSubscriber<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
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
            if let Err(e) = self.sender.try_send(msg) {
                if e.is_disconnected() {
                    // user dropped the handle to the stream, signal removal.
                    return true;
                }
                println!("error {:?}", e)
            }
        }
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl<T: 'static> Subscriber_ for NativeSubscriber<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
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
            if let Err(e) = self.sender.try_send(msg) {
                if e.is_disconnected() {
                    // user dropped the handle to the stream, signal removal.
                    return true;
                }
                println!("error {:?}", e)
            }
        }
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl Subscriber_ for UntypedSubscriber {
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsgUntyped::new_from(&self.topic_type)
            .unwrap_or_else(|_| panic!("no typesupport for {}", self.topic_type));
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
            if let Err(e) = self.sender.try_send(json) {
                if e.is_disconnected() {
                    // user dropped the handle to the stream, signal removal.
                    return true;
                }
                println!("error {:?}", e)
            }
        }
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

pub fn create_subscription_helper(
    node: &mut rcl_node_t,
    topic: &str,
    ts: *const rosidl_message_type_support_t,
    qos_profile: QosProfile,
) -> Result<rcl_subscription_t> {
    let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
    let topic_c_string = CString::new(topic).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let mut subscription_options = rcl_subscription_get_default_options();
        subscription_options.qos = qos_profile.into();
        rcl_subscription_init(
            &mut subscription_handle,
            node,
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
