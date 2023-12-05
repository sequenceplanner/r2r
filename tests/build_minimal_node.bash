#!/bin/bash

# Use the local version of r2r when building the minimal node.
cat >> /r2r/r2r_minimal_node/r2r_minimal_node/Cargo.toml << EOF

[patch.crates-io]
r2r = { path = "../../r2r" }

[workspace]

EOF

# Overwrite r2r_cargo to make sure the version in the r2r repo is ok.
cp /r2r/r2r_cargo.cmake /r2r/r2r_minimal_node/r2r_minimal_node/

# Build using colcon.
cd /r2r/r2r_minimal_node
colcon build
