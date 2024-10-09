//! Tracepoint provider for the `r2r` crate.

use lttng_ust::import_tracepoints;

// Import the tracepoints defined in the build.rs script
import_tracepoints!(concat!(env!("OUT_DIR"), "/r2r_tracepoints.rs"), r2r_tracepoints_internal);

mod tracetools_bindings;

mod rclcpp_tracepoints;
pub use rclcpp_tracepoints::*;

mod r2r_tracepoints;
pub use r2r_tracepoints::*;

mod callback;
pub use callback::Callback;

mod tracing_id;
pub use tracing_id::TracingId;
