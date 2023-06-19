#!/bin/bash

. "$HOME/.cargo/env"

# run rustup to test with latest rust version
rustup update

if [ -e "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi
if [ -e "/opt/ros/galactic/setup.bash" ]; then
    source "/opt/ros/galactic/setup.bash"
fi
if [ -e "/opt/ros/foxy/setup.bash" ]; then
    source "/opt/ros/foxy/setup.bash"
fi

cd /r2r/

"$@"
