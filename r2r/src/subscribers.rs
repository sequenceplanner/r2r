use futures::channel::mpsc;
use std::ffi::CString;

use crate::{error::*, msg_types::*, qos::QosProfile};
use futures::stream::Stream;
use r2r_rcl::*;
use std::ffi::{c_void, CStr};
use std::pin::Pin;
use std::sync::Arc;
use std::task::Poll;
use futures::stream::Stream;

pub trait Subscriber_ {
    fn handle(&self) -> &rcl_subscription_t;
    /// Returns true if the subscriber stream has been dropped.
    fn handle_incoming(&mut self) -> bool;
    // Returns true if the subscriber is waiting for incoming messages
    fn is_waiting(&self) -> bool;
    fn destroy(&mut self, node: &mut rcl_node_t);
}

pub struct TypedSubscriber {
    pub rcl_handle: rcl_subscription_t,
    // The waker to call when new data is available
    pub waker: Arc<std::sync::Mutex<Option<std::task::Waker>>>,
}

// Existing code distinguished these two kinds of subscribers, so keep that distinction in place
// to reduce delta with upstream.
pub type UntypedSubscriber = TypedSubscriber;

pub struct RawSubscriber {
    pub rcl_handle: rcl_subscription_t,
    pub msg_buf: rcl_serialized_message_t,
    pub sender: mpsc::Sender<Vec<u8>>,
}

impl Subscriber_ for TypedSubscriber {
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let locked_waker = self.waker.lock().unwrap();
        if let Some(ref waker) = *locked_waker {
            waker.wake_by_ref();
        }
        false
    }

    fn is_waiting(&self) -> bool {
        self.waker.lock().unwrap().is_some()
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

struct SubscriberStreamWaker {
    waker: Arc<std::sync::Mutex<Option<std::task::Waker>>>,
    pub waiting_state_changed_gc: rcl_guard_condition_t,
}

impl SubscriberStreamWaker {
    fn new(
        waker: Arc<std::sync::Mutex<Option<std::task::Waker>>>, gc: rcl_guard_condition_t,
    ) -> Self {
        SubscriberStreamWaker {
            waker,
            waiting_state_changed_gc: gc,
        }
    }

    fn set_waker(&mut self, waker: std::task::Waker) {
        let was_waiting = {
            let mut stored_waker = self.waker.lock().unwrap();
            let was_waiting = stored_waker.is_some();
            *stored_waker = Some(waker);
            was_waiting
        };

        // If the subscription goes from not-waiting to waiting, notify the waitset so it adds this subscription
        if !was_waiting {
            unsafe {
                match Error::from_rcl_error(rcl_trigger_guard_condition(
                    &mut self.waiting_state_changed_gc,
                )) {
                    Error::RCL_RET_OK => {}
                    e => {
                        // This can only fail if the guard condition object was invalid, so panic is the appropriate response
                        panic!("Failed to trigger guard condition: {e}");
                    }
                }
            }
        }
    }
    fn clear(&mut self) {
        *self.waker.lock().unwrap() = None;
    }
}

pub struct SubscriberStream<T>
where
    T: WrappedTypesupport,
{
    pub rcl_handle: rcl_subscription_t,
    waker: SubscriberStreamWaker,
    // suppress Rust's "unused type" error
    pub stream_type: std::marker::PhantomData<T>,
}

impl<T: WrappedTypesupport + 'static> std::marker::Unpin for SubscriberStream<T> {}
unsafe impl<T: WrappedTypesupport + 'static> std::marker::Send for SubscriberStream<T> {}

pub struct NativeSubscriberStream<T>
where
    T: WrappedTypesupport,
{
    pub stream: SubscriberStream<T>,
}

pub struct UntypedSubscriberStream {
    pub rcl_handle: rcl_subscription_t,
    waker: SubscriberStreamWaker,
    pub topic_type: String,
}

impl std::marker::Unpin for UntypedSubscriberStream {}
unsafe impl std::marker::Send for UntypedSubscriberStream {}

pub struct RawSubscriberStream {
    pub rcl_handle: rcl_subscription_t,
    waker: SubscriberStreamWaker,
    msg_buf: rcl_serialized_message_t,
}

impl std::marker::Unpin for RawSubscriberStream {}
unsafe impl std::marker::Send for RawSubscriberStream {}

impl<T: 'static + WrappedTypesupport> SubscriberStream<T> {
    pub fn new(
        sub: rcl_subscription_t, waker: Arc<std::sync::Mutex<Option<std::task::Waker>>>,
        gc: rcl_guard_condition_t,
    ) -> Self {
        SubscriberStream::<T> {
            rcl_handle: sub,
            waker: SubscriberStreamWaker::new(waker, gc),
            stream_type: std::marker::PhantomData,
        }
    }

    fn receive_native_no_loaning(&mut self) -> Option<WrappedNativeMsg<T>> {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsg::<T>::new();
        let ret = unsafe {
            rcl_take(&self.rcl_handle, msg.void_ptr_mut(), &mut msg_info, std::ptr::null_mut())
        };
        if ret == RCL_RET_OK as i32 {
            Some(msg)
        } else if ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED as i32 {
            // No message available
            None
        } else {
            // An unexpected error while reading. The old code just ignored it.
            // For now just panic, but we should think about this again
            panic!("Error while reading message from subscription: {ret}");
        }
    }

    fn receive(&mut self) -> Option<T> {
        self.receive_native_no_loaning()
            .map(|msg| T::from_native(&msg))
    }

    fn receive_native(&mut self) -> Option<WrappedNativeMsg<T>> {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        unsafe {
            if rcl_subscription_can_loan_messages(&self.rcl_handle) {
                let mut loaned_msg: *mut c_void = std::ptr::null_mut();
                let ret = rcl_take_loaned_message(
                    &self.rcl_handle,
                    &mut loaned_msg,
                    &mut msg_info,
                    std::ptr::null_mut(),
                );
                if ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED as i32 {
                    // no message available
                    return None;
                }
                if ret != RCL_RET_OK as i32 {
                    panic!("Error while reading message from subscription: {ret}");
                }
                let handle_box = Box::new(self.rcl_handle);
                let deallocator = Box::new(|msg: *mut T::CStruct| {
                    let handle_ptr = Box::into_raw(handle_box);
                    let ret =
                        rcl_return_loaned_message_from_subscription(handle_ptr, msg as *mut c_void);
                    if ret != RCL_RET_OK as i32 {
                        let err_str = rcutils_get_error_string();
                        let err_str_ptr = &(err_str.str_) as *const std::os::raw::c_char;
                        let error_msg = CStr::from_ptr(err_str_ptr);

                        let topic_str = rcl_subscription_get_topic_name(handle_ptr);
                        let topic = CStr::from_ptr(topic_str);
                        drop(Box::from_raw(handle_ptr));

                        crate::log_error!(
                            "r2r",
                            "rcl_return_loaned_message_from_subscription() \
                            failed for subscription on topic {}: {}",
                            topic.to_str().expect("to_str() call failed"),
                            error_msg.to_str().expect("to_str() call failed")
                        );
                    }
                    drop(Box::from_raw(handle_ptr));
                });
                Some(WrappedNativeMsg::<T>::from_loaned(loaned_msg as *mut T::CStruct, deallocator))
            } else {
                self.receive_native_no_loaning()
            }
        }
    }
}

impl<T: 'static + WrappedTypesupport> NativeSubscriberStream<T> {
    pub fn new(
        sub: rcl_subscription_t, waker: Arc<std::sync::Mutex<Option<std::task::Waker>>>,
        gc: rcl_guard_condition_t,
    ) -> Self {
        Self {
            stream: SubscriberStream::<T>::new(sub, waker, gc),
        }
    }
}

impl UntypedSubscriberStream {
    pub fn new(
        sub: rcl_subscription_t, waker: Arc<std::sync::Mutex<Option<std::task::Waker>>>,
        gc: rcl_guard_condition_t, topic_type: String,
    ) -> Self {
        Self {
            rcl_handle: sub,
            waker: SubscriberStreamWaker::new(waker, gc),
            topic_type,
        }
    }

    fn receive_json(&mut self) -> Option<Result<serde_json::Value>> {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsgUntyped::new_from(&self.topic_type)
            .unwrap_or_else(|_| panic!("no typesupport for {}", self.topic_type));
        let ret = unsafe {
            rcl_take(&self.rcl_handle, msg.void_ptr_mut(), &mut msg_info, std::ptr::null_mut())
        };
        if ret == RCL_RET_OK as i32 {
            Some(
                msg.to_json()
                    .map_err(|e| crate::Error::SerdeError { err: e.to_string() }),
            )
        } else if ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED as i32 {
            None
        } else {
            panic!("Failed to read from subscription: {ret}");
        }
    }
}

impl RawSubscriberStream {
    fn new(
        sub: rcl_subscription_t, waker: Arc<std::sync::Mutex<Option<std::task::Waker>>>,
        gc: rcl_guard_condition_t,
    ) -> Self {
        Self {
            rcl_handle: sub,
            waker: SubscriberStreamWaker::new(waker, gc),
            msg_buf: unsafe { rcutils_get_zero_initialized_uint8_array() },
        }
    }
    fn receive_raw(&mut self) -> Option<Vec<u8>> {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let ret = unsafe {
            rcl_take_serialized_message(
                &self.rcl_handle,
                &mut self.msg_buf as *mut rcl_serialized_message_t,
                &mut msg_info,
                std::ptr::null_mut(),
            )
        };
        if ret == RCL_RET_OK as i32 {
            Some(if self.msg_buf.buffer == std::ptr::null_mut() {
                Vec::new()
            } else {
                unsafe {
                    std::slice::from_raw_parts(self.msg_buf.buffer, self.msg_buf.buffer_length)
                        .to_vec()
                }
            })
        } else if ret == RCL_RET_SUBSCRIPTION_TAKE_FAILED as i32 {
            None
        } else {
            // An unexpected error while reading. The old code just ignored it.
            // For now just panic, but we should think about this again
            panic!("Error while reading message from subscription: {ret}");
        }
    }
}

impl<T: 'static> Stream for SubscriberStream<T>
where
    T: WrappedTypesupport,
{
    type Item = T;

    // Required method
    fn poll_next(mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> Poll<Option<T>> {
        match self.receive() {
            Some(msg) => {
                self.waker.clear();
                Poll::Ready(Some(msg))
            }
            None => {
                self.waker.set_waker(cx.waker().clone());
                Poll::Pending
            }
        }
    }
}

impl<T: 'static> Stream for NativeSubscriberStream<T>
where
    T: WrappedTypesupport,
{
    type Item = WrappedNativeMsg<T>;

    // Required method
    fn poll_next(
        mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>,
    ) -> Poll<Option<WrappedNativeMsg<T>>> {
        match self.stream.receive_native() {
            Some(msg) => {
                self.stream.waker.clear();
                Poll::Ready(Some(msg))
            }
            None => {
                self.stream.waker.set_waker(cx.waker().clone());
                Poll::Pending
            }
        }
    }
}

impl Stream for UntypedSubscriberStream {
    type Item = Result<serde_json::Value>;

    // Required method
    fn poll_next(
        mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        match self.receive_json() {
            Some(msg) => {
                self.waker.clear();
                Poll::Ready(Some(msg))
            }
            None => {
                self.waker.set_waker(cx.waker().clone());
                Poll::Pending
            }
        }
    }
}

impl Stream for RawSubscriberStream {
    type Item = Vec<u8>;

    // Required method
    fn poll_next(
        mut self: Pin<&mut Self>, cx: &mut std::task::Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        match self.receive_raw() {
            Some(msg) => {
                self.waker.clear();
                Poll::Ready(Some(msg))
            }
            None => {
                self.waker.set_waker(cx.waker().clone());
                Poll::Pending
            }
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

        if let Err(e) = self.sender.try_send(data_bytes) {
            if e.is_disconnected() {
                // user dropped the handle to the stream, signal removal.
                return true;
            }
            log::debug!("error {:?}", e)
        }

        false
    }

    fn is_waiting(&self) -> bool {
        // TODO(tobiasstark): Implement
        true
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
            rcutils_uint8_array_fini(&mut self.msg_buf as *mut rcl_serialized_message_t);
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
