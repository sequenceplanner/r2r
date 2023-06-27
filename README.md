R2R - Easy to use, runtime-agnostic, async rust bindings for ROS2.
====================

Easy to use bindings for ROS2 that do *not* require hooking in to the ROS2 build infrastructure -- `cargo build` is all you need. Convenience Rust types are created by calling into the c introspection libraries. This circumvents the ROS2 .msg/.idl pipeline by relying on already generated C code. By default, the behavior is to build bindings to the RCL and all message types that can be found in the currently sourced ros environment, but it is possible to be more specific to save on build time (see note below).

When integration with the colcon build system is desired, a CMakeLists.txt file can be used to limit the generation of bindings to only include specific (idl) dependencies. This is done through additional environment variables. A minimal example of the colcon integration is available here: <https://github.com/m-dahl/r2r_minimal_node>.

This library differ a bit in style from rclpy and rclcpp as it eliminates all synchronous callbacks in favor of rust futures and streams. Coupled with the rust await syntax, this makes it very pleasant to work with ROS services and actions, even in a single threaded setup (see service.rs example). The library purposefully does not chose an async runtime -- this means that the user needs to take care of any task spawning. This also limits the API to what futures-rs provides.

Documentation can be found on docs.rs: <https://docs.rs/r2r/latest/r2r>.

These bindings are being written organically when things are needed by me and others so please be aware that the API will change. As of now I have no intention of wrapping all of the RCL just for the sake of completeness. However, if you need some specific functionality, feel free to open an issue or a PR!

How to use
--------------------
1. Make sure you have libclang installed. (e.g. libclang-dev on ubuntu)
2. Depend on this package in Cargo.toml: `r2r = "0.7.5"`
3. You need to source your ROS2 installation before building/running.
4. The bindings will rebuild automatically if/when you source your workspace(s).
5. If you make changes to existing message types, run `cargo clean -p r2r_msg_gen` to force recompilation of the rust message types on the next build.

Examples of how to use the crate are included in examples/
```
. /opt/ros/humble/setup.sh
cargo build
cargo run --example subscriber
# In other shell
ros2 topic pub /topic std_msgs/msg/String "data: 'Hello, world'"
```

A note on build times
--------------------
Since the default behavior is to build all sourced message types, build time can quickly become a problem for larger workspaces. To avoid building everything, it is possible to declare only the message packages needed using the environment variable `IDL_PACKAGE_FILTER`. Setting this can be done in `.cargo/config.toml` for convenience, e.g. <https://github.com/m-dahl/r2r_minimal_node/blob/master/r2r_minimal_node/.cargo/config.toml>. Note there is no automatic dependency resolution done for nested message types, so all used message packages need to be explicitly included.

What works?
--------------------
- Up to date with ROS2 ~Dashing~ ~Eloquent~ Foxy Galactic Humble
- Building Rust types
- Publish/subscribe
- Services
- Actions
- Rudimentary parameter handling

Changelog
--------------------
#### [Unreleased]

#### [0.7.5] - 2023-06-27
- Fix issue with snake case conversion for idl header files. <https://github.com/sequenceplanner/r2r/pull/61>
- Fix build issue with race condition between header file generation and calling bindgen. <https://github.com/sequenceplanner/r2r/commit/f5f41baa2554b24813cc2f212ecf2e45476063e1>

#### [0.7.4] - 2023-06-26
- Refactor code generation using syn and quote and improve build times <https://github.com/sequenceplanner/r2r/pull/58>
- Replace (e)println with log <https://github.com/sequenceplanner/r2r/pull/51>

#### [0.7.3] - 2023-06-20
- Fix mistake in docs generation.

#### [0.7.2] - 2023-06-20
- Various CI improvements: <https://github.com/sequenceplanner/r2r/pull/54>, <https://github.com/sequenceplanner/r2r/pull/56>
- Fix build problem at docs.rs <https://github.com/sequenceplanner/r2r/pull/56>
- Clarify compilation flags related to build time in README.md as suggested in <https://github.com/sequenceplanner/r2r/issues/53>

#### [0.7.1] - 2023-05-21
- Add associated constants to generated message types. <https://github.com/sequenceplanner/r2r/pull/46>
- Log loaned message error only once. <https://github.com/sequenceplanner/r2r/pull/44>
- Update r2r_cargo.cmake to latest version (see [here](https://github.com/m-dahl/r2r_minimal_node/commit/897774868edaa97e0272fffc253402d6474aaeb7))

#### [0.7.0] - 2023-03-21
- Use non-mangled names for serde serialization. <https://github.com/sequenceplanner/r2r/pull/40>
- Avoid segfault when rcl_init fails. <https://github.com/sequenceplanner/r2r/pull/41>
- Loaned message support. (Changes `publish_native` api). <https://github.com/sequenceplanner/r2r/pull/42>

#### [0.6.7] - 2023-02-20
- Fix undeclared type on Foxy. <https://github.com/sequenceplanner/r2r/pull/39>

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
