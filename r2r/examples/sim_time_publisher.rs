use r2r::rosgraph_msgs::msg;
use r2r::ClockType::SystemTime;
use r2r::{Clock, QosProfile};
use std::time::Duration;

/// Simple publisher publishing time starting at time 0 every `SENDING_PERIOD`
fn main() -> Result<(), Box<dyn std::error::Error>> {
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
