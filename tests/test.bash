#!/bin/bash

. "$HOME/.cargo/env"

# run rustup to test with latest rust version
rustup update

if [ -e "/opt/ros/iron/setup.bash" ]; then
    source "/opt/ros/iron/setup.bash"
elif [ -e "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
elif [ -e "/opt/ros/galactic/setup.bash" ]; then
    source "/opt/ros/galactic/setup.bash"
elif [ -e "/opt/ros/foxy/setup.bash" ]; then
    source "/opt/ros/foxy/setup.bash"
fi

cd /r2r/

"$@"
