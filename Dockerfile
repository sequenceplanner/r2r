FROM kristoferb/spbase_ros2:rolling

COPY . /
CMD cargo test --verbose
