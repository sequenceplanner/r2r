use crate::{tracetools_bindings as tp, TracingId};
use r2r_rcl::{rcl_node_t, rcl_service_t, rcl_subscription_t, rcl_timer_t};
use std::{ffi::CString, ptr::null};

// Documentation of tracepoints is based on github:ros2/ros2_tracing project
// TODO: Check that all references and pointers must be stable (not change location)

const fn ref_to_c_void<T>(t: &T) -> *const std::ffi::c_void {
    std::ptr::from_ref(t).cast()
}

macro_rules! c_void {
    ($e:ident) => {
        $e as *const std::ffi::c_void
    };
}

/// Message publication.
///
/// Notes the pointer to the `message` being published at the `rclcpp`/`r2r` level.
/// The message pointer is NOT dereferenced.
///
/// Tracepoint: `ros2::rclcpp_publish`
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn trace_publish(message: *const std::ffi::c_void) {
    unsafe {
        // first argument documentation:
        // publisher_handle not used, but kept for API/ABI stability
        tp::ros_trace_rclcpp_publish(null(), message);
    }
}

/// Subscription object initialisation.
///
/// Links the `subscription_handle` from `rcl` to the addresss of rust `subscription` reference.
/// There can be more than 1 `subscription` for 1 `subscription_handle`.
///
/// Tracepoint: `ros2::rclcpp_subscription_init`
pub fn trace_subscription_init<S>(
    subscription_handle: *const rcl_subscription_t, subscription: &S,
) {
    unsafe {
        tp::ros_trace_rclcpp_subscription_init(
            c_void!(subscription_handle),
            ref_to_c_void(subscription),
        );
    }
}

/// Link a subscription callback identified by `callback_id` to a `subscription` object.
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
/// Notes the **reference** to the `message` being taken at the `rclcpp`/`r2r` level.
///
/// To trace void pointer to the message use [`trace_take_ptr`].
///
/// Tracepoint: `ros2::rclcpp_take`
pub fn trace_take<M>(message: &M) {
    unsafe {
        tp::ros_trace_rclcpp_take(ref_to_c_void(message));
    }
}

/// Message taking.
///
/// Notes the **void pointer** to the `message` being taken at the `rclcpp`/`r2r` level.
/// The message pointer is NOT dereferenced.
///
/// To trace reference to the message use [`trace_take`].
///
/// Tracepoint: `ros2::rclcpp_take`
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub fn trace_take_ptr(message: *const std::ffi::c_void) {
    unsafe {
        tp::ros_trace_rclcpp_take(message);
    }
}

/// Link a service callback identified by `callback_id` to a `service`.
///
/// Tracepoint: `ros2::rclcpp_service_callback_added`
pub fn trace_service_callback_added(service: *const rcl_service_t, callback_id: usize) {
    unsafe { tp::ros_trace_rclcpp_service_callback_added(c_void!(service), c_void!(callback_id)) }
}

/// Link a timer callback identified by `callback_id` to its `rcl_timer_t` handle.
///
/// Tracepoint: `ros2::rclcpp_timer_callback_added`
pub fn trace_timer_callback_added(timer: TracingId<rcl_timer_t>, callback_id: usize) {
    unsafe {
        tp::ros_trace_rclcpp_timer_callback_added(timer.c_void(), c_void!(callback_id));
    }
}

/// Link a `timer` to a `node`.
///
/// Tracepoint: `ros2::rclcpp_timer_link_node`
pub fn trace_timer_link_node(timer: TracingId<rcl_timer_t>, node: TracingId<rcl_node_t>) {
    unsafe {
        tp::ros_trace_rclcpp_timer_link_node(timer.c_void(), node.c_void());
    }
}

/// Register a demangled `function_symbol` with a `callback_id`.
///
/// Tracepoint: `ros2::callback_register`
pub fn trace_callback_register(callback_id: usize, function_symbol: &str) {
    let function_symbol = CString::new(function_symbol)
        .expect("r2r tracing: Cannot convert function_symbol to CString");

    unsafe {
        tp::ros_trace_rclcpp_callback_register(c_void!(callback_id), function_symbol.as_ptr());
    }
}

/// Start of a callback
///
/// If this callback is done via intra-process set `is_intra_process` to `true`.
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

/// Notes the start time of the executor phase that gets the next executable that's ready.
///
/// Tracepoint: `ros2::rclcpp_executor_get_next_ready`
pub fn trace_executor_get_next_ready() {
    unsafe {
        tp::ros_trace_rclcpp_executor_get_next_ready();
    }
}

/// Notes the start time of the executor phase that waits for work and notes the `timeout` value.
///
/// Tracepoint: `ros2::rclcpp_executor_wait_for_work`
pub fn trace_executor_wait_for_work(timeout: i64) {
    unsafe {
        tp::ros_trace_rclcpp_executor_wait_for_work(timeout);
    }
}

/// Executable execution.
///
/// Notes an executable being executed using its `rcl_handle`, which can be a:
/// * timer
/// * subscription
///
/// Tracepoint: `ros2::rclcpp_executor_execute`
pub fn trace_executor_execute<H>(rcl_handle: *const H) {
    unsafe {
        tp::ros_trace_rclcpp_executor_execute(c_void!(rcl_handle));
    }
}
