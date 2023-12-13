use futures::channel::mpsc;
use std::ffi::CString;

use crate::error::*;
use crate::msg_types::*;
use crate::qos::QosProfile;
use r2r_rcl::*;
use std::ffi::c_void;
use std::ffi::CStr;

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
                    drop(Box::from_raw(handle_ptr));
                    if ret != RCL_RET_OK as i32 {
                        let err_str = rcutils_get_error_string();
                        let err_str_ptr = &(err_str.str_) as *const std::os::raw::c_char;
                        let error_msg = CStr::from_ptr(err_str_ptr);

                        let topic_str = rcl_subscription_get_topic_name(handle_ptr);
                        let topic = CStr::from_ptr(topic_str);

                        panic!(
                            "rcl_return_loaned_message_from_subscription() \
                            failed for subscription on topic {}: {}",
                            topic.to_str().expect("to_str() call failed"),
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

        // This code is based on:
        //
        // https://github.com/josephduchesne/rclpy/blob/502e2135498460dd4c74cf3a6fa543590364a1fe/rclpy/src/rclpy/_rclpy.c#L2612-L2649

        let mut msg_info = rmw_message_info_t::default(); // we dont care for now

        let mut msg: rcl_serialized_message_t = unsafe { rcutils_get_zero_initialized_uint8_array() };
        let allocator: rcutils_allocator_t = unsafe { rcutils_get_default_allocator() };
        let ret: rcl_ret_t = unsafe {
            rcutils_uint8_array_init(&mut msg as *mut rcl_serialized_message_t, 0, &allocator)
        };

        if ret != RCL_RET_OK as i32 {
            log::error!("Failed to initialize message: {:?}", unsafe { rcutils_get_error_string().str_ });
            unsafe { rcutils_reset_error() };

            let r_fini: rmw_ret_t = unsafe { 
                rcutils_uint8_array_fini(&mut msg as *mut rcl_serialized_message_t)
            };

            if r_fini != RMW_RET_OK as i32 {
                log::error!("Failed to deallocate message buffer: {r_fini}");
            }
            return false;
          }

        let ret = unsafe {
            rcl_take_serialized_message(
                &self.rcl_handle,
                &mut msg as *mut rcl_serialized_message_t,
                &mut msg_info, 
                std::ptr::null_mut())
        };
        if ret != RCL_RET_OK as i32 {
            log::error!(
                "Failed to take_serialized from a subscription: {:?}", 
                unsafe { rcutils_get_error_string().str_ });

            //   rcl_reset_error();
            unsafe { rcutils_reset_error() };

            let r_fini: rmw_ret_t = unsafe {
                rcutils_uint8_array_fini(&mut msg as *mut rcl_serialized_message_t)
            };

            if r_fini != RMW_RET_OK as i32 {
                log::error!("Failed to deallocate message buffer: {r_fini}");
            }

            return false;
    }

        // TODO put rcutils_uint8_array_fini in a message drop guard?
        //
        // Or is is safe to deallocate with Vec::drop instead of rcutils_uint8_array_fini?

        // let data_bytes = unsafe {
        //     Vec::from_raw_parts(msg.buffer, msg.buffer_length, msg.buffer_capacity)
        // };

        let data_bytes =  unsafe {
            std::slice::from_raw_parts(msg.buffer, msg.buffer_length).to_vec()
        };

        let r_fini: rmw_ret_t  = unsafe {
            rcutils_uint8_array_fini(&mut msg as *mut rcl_serialized_message_t)
        };

        if r_fini != RMW_RET_OK as i32 {
            log::error!("Failed to deallocate message buffer: {r_fini}");

            return false;
        }

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
        }
    }
}

pub fn create_subscription_helper(
    node: &mut rcl_node_t, topic: &str, ts: *const rosidl_message_type_support_t,
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

