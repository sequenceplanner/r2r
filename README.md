R2R - Easy to use, runtime-agnostic, async rust bindings for ROS2.
====================

Easy to use bindings for ROS2 that do *not* require hooking in to the ROS2 build infrastructure -- `cargo build` is all you need. Convenience Rust types are created by calling into the c introspection libraries. This circumvents the ROS2 .msg/.idl pipeline by relying on already generated C code. By default, the behavior is to build bindings to the RCL and all message types that can be found in the currently sourced ros environment.

When integration with the colcon build system is desired, a CMakeLists.txt file can be used to limit the generation of bindings to only include specific (idl) dependencies. This is done through additional environment variables. A minimal example of the colcon integration is available here: <https://github.com/m-dahl/r2r_minimal_node/>.

This library differ a bit in style from rclpy and rclcpp as it eliminates all synchronous callbacks in favor of rust futures and streams. Coupled with the rust await syntax, this makes it very pleasant to work with ROS services and actions, even in a single threaded setup (see service.rs example). The library purposefully does not chose an async runtime -- this means that the user needs to take care of any task spawning. This also limits the API to what futures-rs provides.

Manual is available on github pages <https://sequenceplanner.github.io/r2r/> (documention is lacking though).

These bindings are being written organically when things are needed by me and others so please be aware that the API will change. As of now I have no intention of wrapping all of the RCL just for the sake of completeness. However, if you need some specific functionality, feel free to open an issue or a PR!

How to use
--------------------
1. Make sure you have libclang installed. (e.g. libclang-dev on ubuntu)
2. Depend on this package in Cargo.toml: r2r = "0.6.3"
3. You need to source your ROS2 installation before building/running.
4. The bindings will rebuild automatically if/when you source your workspace(s).
5. If you make changes to existing message types, run cargo clean -p msg_gen to force recompilation of the rust message types on the next build.

Examples of how to use the crate are included in examples/
```
. /opt/ros/foxy/setup.sh
cargo build
cargo run --example subscriber
# In other shell
ros2 topic pub /topic std_msgs/msg/String "data: 'Hello, world'"
```

What works?
--------------------
- Up to date with ROS2 ~Dashing~ ~Eloquent~ Foxy Galactic Humble
- Building Rust types
- Publish/subscribe
- Services
- Actions
- Rudimentary parameter handling

TODO
--------------------
- Documentation is lacking. (For now, look at the examples.)


LICENSING
--------------------
All code is under the MIT licence unless another license is specified in the source files. Some parts from the rclrust (https://github.com/rclrust/rclrust) package are included in this repo and these are licensed under Apache 2.0.


***
<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.
