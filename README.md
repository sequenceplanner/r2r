R2R - Minimal ROS2 Rust bindings
=============

Minimal bindings for ROS2 that does *not* require hooking in to the ROS2 build infrastructure. If you want a more ROS-oriented approach, see <https://github.com/ros2-rust/ros2_rust>. In these bindings, convenience Rust types are created by calling into the c introspection libraries to circumvent the .msg/.idl pipeline. The convenience types can be ignored when you need to trade convenience for performance, e.g. treating large chunks of data manually.

How to use
------------
1. Depend on this package: r2r = { git = "https://github.com/sequenceplanner/r2r" }.
2. You need to source your ROS2 installation before building/running.
3. The bindings will rebuild automatically if/when you source your workspaces.

A couple of examples are included in examples/
```
. /opt/ros/dashing/setup.sh
cargo build
cargo run --example subscriber_with_thread
```


What works?
--------
- Up to date with ROS2 Dashing
- Building Rust types
- Publish/subscribe

TODO
------------
- The code generation is currently just a big hack. Needs cleanup and refactoring.
- Implement error handling. Now all methods just return Err(()).
- Expose more of the RCL.
- Services and action types are currently ignored.
- QoS settings etc.
