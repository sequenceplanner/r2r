use crate::{tracepoint_fn, TracingId};
use r2r_rcl::{rcl_node_t, rcl_service_t, rcl_subscription_t, rcl_timer_t};

#[cfg(feature = "tracing")]
use crate::tracetools_bindings as tp;
#[cfg(feature = "tracing")]
use std::{ffi::CString, ptr::null};

#[cfg(feature = "tracing")]
const fn ref_to_c_void<T>(t: &T) -> *const std::ffi::c_void {
    std::ptr::from_ref(t).cast()
}

#[cfg(feature = "tracing")]
macro_rules! c_void {
    ($e:ident) => {
        ($e) as *const std::ffi::c_void
    };
}

// Documentation of tracepoints is based on https://github.com/ros2/ros2_tracing project.
// From file `tracetools/include/tracetools/tracetools.h`

tracepoint_fn! {
/// Message publication.
///
/// Records the pointer to the `message` being published at the `rclcpp`/`r2r` level.
///
/// Tracepoint: `ros2::rclcpp_publish
// Lint allow note: The message pointer is NOT dereferenced.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn trace_publish(message: *const std::ffi::c_void) {
    unsafe {
        // first argument documentation:
        // publisher_handle not used, but kept for API/ABI stability
        tp::ros_trace_rclcpp_publish(null(), message);
    }
}

/// Subscription object initialization.
///
/// Tracepoint to allow associating the `subscription_handle` from `rcl` with the address of rust `subscription` reference.
/// There can be more than 1 `subscription` for 1 `subscription_handle`.
///
/// Tracepoint: `ros2::rclcpp_subscription_init`
pub fn trace_subscription_init<S>(
    subscription_handle: *const rcl_subscription_t, subscription: &S,
) {
    unsafe {
        tp::ros_trace_rclcpp_subscription_init(
            subscription_handle.cast(),
            ref_to_c_void(subscription),
        );
    }
}

/// Tracepoint to allow associating the subscription callback identified by `callback_id` with the `subscription` object.
///
/// Tracepoint: `ros2::rclcpp_subscription_callback_added`
pub fn trace_subscription_callback_added<S>(subscription: &S, callback_id: usize) {
    unsafe {
        tp::ros_trace_rclcpp_subscription_callback_added(
            ref_to_c_void(subscription),
            c_void!(callback_id),
        );
    }
}

/// Message taking.
///
/// Records the **reference** to the `message` being taken at the `rclcpp`/`r2r` level.
///
/// To trace messages pointed to by void pointer use [`trace_take_ptr`].
///
/// Tracepoint: `ros2::rclcpp_take`
pub fn trace_take<M>(message: &M) {
    unsafe {
        tp::ros_trace_rclcpp_take(ref_to_c_void(message));
    }
}

/// Message taking.
///
/// Records the **void pointer** to the `message` being taken at the `rclcpp`/`r2r` level.
///
/// To trace messages by their reference use [`trace_take`].
///
/// Tracepoint: `ros2::rclcpp_take`
// Lint allow note: The message pointer is NOT dereferenced.
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn trace_take_ptr(message: *const std::ffi::c_void) {
    unsafe {
        tp::ros_trace_rclcpp_take(message);
    }
}

/// Tracepoint to allow associating the service callback identified by `callback_id` with a `service`.
///
/// Tracepoint: `ros2::rclcpp_service_callback_added`
pub fn trace_service_callback_added(service: *const rcl_service_t, callback_id: usize) {
    unsafe { tp::ros_trace_rclcpp_service_callback_added(service.cast(), c_void!(callback_id)) }
}

/// Tracepoint to allow associating the timer callback identified by `callback_id` with its `rcl_timer_t` handle.
///
/// Tracepoint: `ros2::rclcpp_timer_callback_added`
pub fn trace_timer_callback_added(timer: TracingId<rcl_timer_t>, callback_id: usize) {
    unsafe {
        tp::ros_trace_rclcpp_timer_callback_added(timer.c_void(), c_void!(callback_id));
    }
}

/// Tracepoint to allow associating the `timer` with a `node`.
///
/// Tracepoint: `ros2::rclcpp_timer_link_node`
pub fn trace_timer_link_node(timer: TracingId<rcl_timer_t>, node: TracingId<rcl_node_t>) {
    unsafe {
        tp::ros_trace_rclcpp_timer_link_node(timer.c_void(), node.c_void());
    }
}

/// Tracepoint to allow associating demangled `function_symbol` with a `callback_id`.
///
/// Allocates memory to store the function symbol as a `CString`.
///
/// Tracepoint: `ros2::callback_register`
///
/// # Panics
/// If `function_symbol` contains a null byte.
pub fn trace_callback_register(callback_id: usize, function_symbol: &str) {
    let function_symbol = CString::new(function_symbol)
        .expect("r2r tracing: Cannot convert function_symbol to CString. It contains null byte.");

    unsafe {
        tp::ros_trace_rclcpp_callback_register(c_void!(callback_id), function_symbol.as_ptr());
    }
}

/// Start of a callback
///
/// Set `is_intra_process` depending on whether this callback is done via intra-process or not
///
/// Tracepoint: `ros2::callback_start`
pub fn trace_callback_start(callback_id: usize, is_intra_process: bool) {
    unsafe {
        tp::ros_trace_callback_start(c_void!(callback_id), is_intra_process);
    }
}

/// End of a callback.
///
/// Tracepoint: `ros2::callback_end`
pub fn trace_callback_end(callback_id: usize) {
    unsafe {
        tp::ros_trace_callback_end(c_void!(callback_id));
    }
}
}
