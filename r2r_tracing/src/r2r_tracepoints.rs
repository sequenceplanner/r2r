use crate::r2r_tracepoints_internal::r2r as tp;
use r2r_rcl::{rcl_node_t, rcl_subscription_t};
use std::time::Duration;

/// The `node` started spinning with given `timeout`.
pub fn trace_spin_start(node: *const rcl_node_t, timeout: Duration) {
    let timeout_s = timeout.as_secs();
    let timeout_ns = timeout.subsec_nanos();

    tp::spin_start(node as usize, timeout_s, timeout_ns);
}

/// The `node` ended spinning.
pub fn trace_spin_end(node: *const rcl_node_t) {
    tp::spin_end(node as usize);
}

/// The `node`` timeouted while spinning
pub fn trace_spin_timeout(node: *const rcl_node_t) {
    tp::spin_timeout(node as usize);
}

/// `subscriber` received update time message with new time `time_s` seconds and `time_ns` nanoseconds.
pub fn trace_update_time(subscriber: *const rcl_subscription_t, time_s: i32, time_ns: u32) {
    tp::update_time(subscriber as usize, time_s, time_ns);
}
