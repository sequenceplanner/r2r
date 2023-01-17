#!/bin/bash

source /opt/ros/humble/setup.bash
source /opt/ros/galactic/setup.bash
rustup update

cd /r2r/
/root/.cargo/bin/cargo test
