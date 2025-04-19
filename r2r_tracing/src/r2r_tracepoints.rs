#[cfg(feature = "tracing")]
use crate::r2r_tracepoints_bindings::r2r as tp;
use crate::tracepoint_fn;
use r2r_rcl::rcl_node_t;
use std::time::Duration;

tracepoint_fn! {
/// The `node` started spinning with given `timeout`.
pub fn trace_spin_start(node: *const rcl_node_t, timeout: Duration) {
    let timeout_s = timeout.as_secs();
    let timeout_ns = timeout.subsec_nanos();

    tp::spin_start(node as usize, timeout_s, timeout_ns);
}

/// The `node` ended spinning function.
///
/// If the spinning function ended by a timeout, use `trace_spin_timeout` instead.
pub fn trace_spin_end(node: *const rcl_node_t) {
    tp::spin_end(node as usize);
}

/// The `node` woke up from waiting on a wait set without reaching timeout.
pub fn trace_spin_wake(node: *const rcl_node_t) {
    tp::spin_wake(node as usize);
}

/// The `node` timeouted while spinning
pub fn trace_spin_timeout(node: *const rcl_node_t) {
    tp::spin_timeout(node as usize);
}
}
