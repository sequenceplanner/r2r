#!/bin/bash

# run commands in root
cd `git rev-parse --show-toplevel`

# remove old bindings
rm r2r*/bindings/*

# only refresh these packages
export IDL_PACKAGE_FILTER='action_msgs;diagnostic_msgs;geometry_msgs;lifecycle_msgs;map_msgs;move_base_msgs;nav_msgs;pendulum_msgs;rosgraph_msgs;sensor_msgs;shape_msgs;statistics_msgs;std_msgs;stereo_msgs;test_msgs;tf2_geometry_msgs;tf2_msgs;trajectory_msgs;unique_identifier_msgs;visualization_msgs'

# refresh bindings
cargo clean
cargo build --features save-bindgen

# make sure the bindings work without ros
cargo clean
env -u AMENT_PREFIX_PATH -u ROS_DISTRO cargo build --features doc-only
