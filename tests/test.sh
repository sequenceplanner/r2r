#!/bin/bash

# rustup to test with latest rust version
rustup update

cd /r2r/
/root/.cargo/bin/cargo test
