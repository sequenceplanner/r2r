use r2r;
use failure::Error;

/// try to run like this
/// cargo run --example logging -- --ros-args --log-level DEBUG
/// The logs produced with the node logger should show up in /rosout

fn main() -> Result<(), Error> {
    r2r::log_debug!("before_init", "debug msg");
    let ctx = r2r::Context::create()?;


    let node = r2r::Node::create(ctx, "logger_node", "")?;

    let mut i = 0;
    loop {
        r2r::log_debug!(node.logger(), "debug msg: {}", i as f64 / 2.5);
        std::thread::sleep_ms(10);
        r2r::log_info!(node.logger(), "info msg {}", i % 2);
        std::thread::sleep_ms(10);
        r2r::log_warn!(node.logger(), "warn msg {:?}", i.to_string());
        std::thread::sleep_ms(10);
        r2r::log_error!(node.logger(), "error msg {:?}", i.to_string().as_bytes());
        std::thread::sleep_ms(10);
        r2r::log_fatal!(node.logger(), "fatal msg {}", i);

        std::thread::sleep_ms(1000);

        r2r::log_debug!("other_logger", "i = {}", i);
        i+=1;
    }
}
