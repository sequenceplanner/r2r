use r2r::{Clock, ClockType::SystemTime, QosProfile};
use std::time::Duration;

/// Simple publisher publishing time starting at time 0 every `SENDING_PERIOD`
#[cfg(r2r__rosgraph_msgs__msg__Clock)]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    use r2r::rosgraph_msgs::msg;
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "clock_publisher", "")?;
    let qos = QosProfile::default().keep_last(1);
    let publisher = node.create_publisher("/clock", qos)?;

    const SENDING_PERIOD: Duration = Duration::from_millis(100);
    const SIM_TIME_MULTIPLIER: f64 = 0.1;

    let mut clock = Clock::create(SystemTime)?;
    let zero_time = clock.get_now()?;
    let mut msg = msg::Clock::default();

    loop {
        let time_diff = clock.get_now()? - zero_time;
        let time = time_diff.mul_f64(SIM_TIME_MULTIPLIER);
        msg.clock = Clock::to_builtin_time(&time);

        publisher.publish(&msg)?;
        println!("Publishing time {}.{:9} s", time.as_secs(), time.subsec_nanos());

        std::thread::sleep(SENDING_PERIOD);
    }
}

#[cfg(not(r2r__rosgraph_msgs__msg__Clock))]
fn main() {
    panic!("Sim_time_publisher example is not compiled with 'rosgraph_msgs'.");
}
