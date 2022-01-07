use r2r::builtin_interfaces::msg::Duration;
use r2r::std_msgs::msg::Int32;
use r2r::trajectory_msgs::msg::*;
use r2r::QosProfile;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let publisher = node.create_publisher::<JointTrajectoryPoint>("/jtp", QosProfile::default())?;
    let publisher2 = node.create_publisher::<Int32>("/native_count", QosProfile::default())?;

    // run for 10 seconds
    let mut count = 0;
    let mut positions: Vec<f64> = Vec::new();
    while count < 100 {
        positions.push(count as f64);
        let to_send = JointTrajectoryPoint {
            positions: positions.clone(),
            time_from_start: Duration {
                sec: count,
                nanosec: 0,
            },
            ..Default::default()
        };
        let mut native = r2r::NativeMsg::<Int32>::new();
        native.data = count;

        publisher.publish(&to_send).unwrap();
        publisher2.publish_native(&native).unwrap();

        std::thread::sleep(std::time::Duration::from_millis(100));
        count += 1;
    }

    println!("All done!");

    Ok(())
}
