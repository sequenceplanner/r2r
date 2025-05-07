# r2r_tracing

Internal dependency containing tracepoint definitions or imports from `tracetools` ROS package for r2r.

Uses LTTng tracing framework.

## Feature flag `tracing`

The crate will generate and link tracepoint libraries only when the feature flag is enabled.

Without specifying the feature flag `tracing` all exported functions are No-ops.

## Depends on

- `tracetools` ROS package
  - This package is a part of ROS distribution.
  - `r2r_tracing` dynamically loads `tracetools` library to obtain tracepoints used by `rclcpp`.
- `lttng-ust` crate
  - To define additional tracepoints.

## Recording traces of R2R applications

Make sure to enable feature flag `tracing`.

Then start tracing session:

- Either by installing [`ros2trace`](https://index.ros.org/p/ros2trace/) and running:

  ```sh
  ros2 trace -u 'ros2:*' 'r2r:*'
  ```

  The traces will be available in `$HOME/.ros/tracing/session-<timestamp>`.

- Alternatively, you can trace your application directly with LTTng:

  ```sh
  # Session name is an optional user-chosen name for the trace
  lttng create [session-name]
  lttng enable-event -u 'ros2:*,r2r:*'
  lttng add-context -u --type=vtid --type=vpid --type=procname
  lttng start
  # Start the ROS system here.
  # Let it run for as long as you want to trace it.
  lttng destroy
  ```

  The traces will be available in `$HOME/lttng-traces/<session-name>-<timestamp>`.
