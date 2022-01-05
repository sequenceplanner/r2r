/// try to run like this
/// cargo run --example logging -- --ros-args --log-level DEBUG
/// The logs produced with the node logger should show up in /rosout

fn main() -> Result<(), Box<dyn std::error::Error>> {
    r2r::log_debug!("before_init", "debug msg");
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "logger_node", "")?;
    let nl = node.logger();

    let mut i = 0;
    loop {
        match i % 5 {
            0 => r2r::log_debug!(nl, "debug msg: {}", i as f64 / 2.5),
            1 => r2r::log_info!(nl, "info msg {}", i % 2),
            2 => r2r::log_warn!(nl, "warn msg {:?}", i.to_string()),
            3 => r2r::log_error!(nl, "error msg {:?}", i.to_string().as_bytes()),
            _ => r2r::log_fatal!(nl, "fatal msg {:#X}", i),
        }

        std::thread::sleep(std::time::Duration::from_millis(1000));

        // non-node logger only outputs to stdout
        r2r::log_debug!("other_logger", "i = {}", i);
        i += 1;
    }
}
