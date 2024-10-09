use futures::channel::mpsc;
use std::ffi::CString;

use crate::{error::*, msg_types::*, qos::QosProfile};
use r2r_rcl::*;
use std::ffi::{c_void, CStr};

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

pub struct RawSubscriber {
    pub rcl_handle: rcl_subscription_t,
    pub msg_buf: rcl_serialized_message_t,
    pub sender: mpsc::Sender<Vec<u8>>,
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
            rcl_take(&self.rcl_handle, msg.void_ptr_mut(), &mut msg_info, std::ptr::null_mut())
        };
        if ret == RCL_RET_OK as i32 {
            r2r_tracing::trace_take_ptr(msg.void_ptr());

            let msg = T::from_native(&msg);
            if let Err(e) = self.sender.try_send(msg) {
                if e.is_disconnected() {
                    // user dropped the handle to the stream, signal removal.
                    return true;
                }
                log::debug!("error {:?}", e)
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
        let msg = unsafe {
            if rcl_subscription_can_loan_messages(&self.rcl_handle) {
                let mut loaned_msg: *mut c_void = std::ptr::null_mut();
                let ret = rcl_take_loaned_message(
                    &self.rcl_handle,
                    &mut loaned_msg,
                    &mut msg_info,
                    std::ptr::null_mut(),
                );
                if ret != RCL_RET_OK as i32 {
                    return false;
                }
                let handle_box = Box::new(self.rcl_handle);
                let deallocator = Box::new(|msg: *mut T::CStruct| {
                    let handle_ptr = Box::into_raw(handle_box);
                    let ret =
                        rcl_return_loaned_message_from_subscription(handle_ptr, msg as *mut c_void);
                    if ret == RCL_RET_OK as i32 {
                        drop(Box::from_raw(handle_ptr));
                    } else {
                        let topic_str = rcl_subscription_get_topic_name(handle_ptr);
                        let topic = CStr::from_ptr(topic_str).to_str().expect("to_str() call failed").to_owned();
                        drop(Box::from_raw(handle_ptr));

                        let err_str = rcutils_get_error_string();
                        let err_str_ptr = &(err_str.str_) as *const std::os::raw::c_char;
                        let error_msg = CStr::from_ptr(err_str_ptr);

                        // Returning a loan shouldn't fail unless one of the handles or pointers
                        // is invalid, both of which indicate a severe bug. Panicking is therefore
                        // more appropriate than leaking the loaned message.
                        panic!(
                            "rcl_return_loaned_message_from_subscription() \
                            failed for subscription on topic {}: {}",
                            topic,
                            error_msg.to_str().expect("to_str() call failed")
                        );
                    }
                });
                WrappedNativeMsg::<T>::from_loaned(loaned_msg as *mut T::CStruct, deallocator)
            } else {
                let mut new_msg = WrappedNativeMsg::<T>::new();
                let ret = rcl_take(
                    &self.rcl_handle,
                    new_msg.void_ptr_mut(),
                    &mut msg_info,
                    std::ptr::null_mut(),
                );
                if ret != RCL_RET_OK as i32 {
                    return false;
                }
                new_msg
            }
        };

        r2r_tracing::trace_take_ptr(msg.void_ptr());

        if let Err(e) = self.sender.try_send(msg) {
            if e.is_disconnected() {
                // user dropped the handle to the stream, signal removal.
                return true;
            }
            log::error!("error {:?}", e)
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
            rcl_take(&self.rcl_handle, msg.void_ptr_mut(), &mut msg_info, std::ptr::null_mut())
        };
        if ret == RCL_RET_OK as i32 {
            r2r_tracing::trace_take_ptr(msg.void_ptr());

            let json = msg.to_json();
            if let Err(e) = self.sender.try_send(json) {
                if e.is_disconnected() {
                    // user dropped the handle to the stream, signal removal.
                    return true;
                }
                log::debug!("error {:?}", e)
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

impl Subscriber_ for RawSubscriber {
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let ret = unsafe {
            rcl_take_serialized_message(
                &self.rcl_handle,
                &mut self.msg_buf as *mut rcl_serialized_message_t,
                &mut msg_info,
                std::ptr::null_mut(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            log::error!("failed to take serialized message");
            return false;
        }

        let data_bytes = if self.msg_buf.buffer == std::ptr::null_mut() {
            Vec::new()
        } else {
            unsafe {
                std::slice::from_raw_parts(self.msg_buf.buffer, self.msg_buf.buffer_length).to_vec()
            }
        };

        r2r_tracing::trace_take(&self.msg_buf);

        if let Err(e) = self.sender.try_send(data_bytes) {
            if e.is_disconnected() {
                // user dropped the handle to the stream, signal removal.
                return true;
            }
            log::debug!("error {:?}", e)
        }

        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
            rcutils_uint8_array_fini(&mut self.msg_buf as *mut rcl_serialized_message_t);
        }
    }
}

pub unsafe fn create_subscription_helper(
    subscription_handle: &mut rcl_subscription_t, node: &mut rcl_node_t, topic: &str,
    ts: *const rosidl_message_type_support_t, qos_profile: QosProfile,
) -> Result<()> {
    let topic_c_string = CString::new(topic).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let mut subscription_options = rcl_subscription_get_default_options();
        subscription_options.qos = qos_profile.into();
        rcl_subscription_init(
            subscription_handle,
            node,
            ts,
            topic_c_string.as_ptr(),
            &subscription_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(())
    } else {
        Err(Error::from_rcl_error(result))
    }
}
