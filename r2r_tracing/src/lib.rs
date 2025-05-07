//! Tracepoint provider for the `r2r` crate.

#[cfg(feature = "tracing")]
mod r2r_tracepoints_bindings;

#[cfg(feature = "tracing")]
mod tracetools_bindings;

mod rclcpp_tracepoints;
pub use rclcpp_tracepoints::*;

mod r2r_tracepoints;
pub use r2r_tracepoints::*;

mod tracing_id;
pub use tracing_id::TracingId;

mod macros;
use macros::tracepoint_fn;
