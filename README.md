R2R - Minimal ROS2 Rust bindings
====================

Minimal bindings for ROS2 that do *not* require hooking in to the ROS2 build infrastructure -- `cargo build` is all you need. Convenience Rust types are created by calling into the c introspection libraries. This circumvents the ROS2 .msg/.idl pipeline by relying on already generated C code. The convenience types can be ignored when you need to trade convenience for performance, e.g. treating large chunks of data manually. By default, the behavior is to build bindings to the RCL and all message types that can be found in the currently sourced ros environment.

When integration with the colcon build system is desired, a CMakeLists.txt file can be used to limit the generation of bindings to only include specific (idl) dependencies. This is done through additional environment variables. A minimal example is available here: <https://github.com/m-dahl/r2r_minimal_node/>.

Manual is available on github pages <https://sequenceplanner.github.io/r2r/>

How to use
--------------------
1. Make sure you have libclang installed. (e.g. libclang-dev on ubuntu)
2. Depend on this package: r2r = { git = "https://github.com/sequenceplanner/r2r" }.
3. You need to source your ROS2 installation before building/running.
4. The bindings will rebuild automatically if/when you source your workspace(s).
5. If you make changes to existing message types, run cargo clean -p msg_gen to force recompilation of the rust message types on the next build.

Examples of how to use the crate are included in examples/
```
. /opt/ros/foxy/setup.sh
cargo build
cargo run --example subscriber_with_thread
# In other shell
ros2 topic pub /hi std_msgs/msg/String "data: 'Hello, world!'"
```

An example application can be found here <https://github.com/sequenceplanner/r2r-echo>, which can be installed by running cargo install --git https://github.com/sequenceplanner/r2r-echo.

What works?
--------------------
- Up to date with ROS2 ~Dashing~ ~Eloquent~ Foxy Galactic
- Building Rust types
- Publish/subscribe
- Services
- Actions
- Static parameters (e.g. loading from yaml files and parsing launch parameters)

TODO
--------------------
- Implement the services associated with updating parameters at run-time.
- Documentation. (For now, look at the examples.)
- General cleanup and error handling.
- Expose more of the RCL like QoS settings.

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
