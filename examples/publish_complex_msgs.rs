use r2r;
use r2r::builtin_interfaces::msg::Duration;
use r2r::std_msgs::msg::Int32;
use r2r::trajectory_msgs::msg::*;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let publisher = node.create_publisher::<JointTrajectoryPoint>("/hej")?;
    let publisher2 = node.create_publisher::<Int32>("/native_count")?;

    let mut c = 0;
    let mut positions: Vec<f64> = Vec::new();
    let cb = move |x: r2r::std_msgs::msg::String| {
        println!("at count {} got: {}", c, x.data);
        c = c + 1;
        positions.push(c as f64);
        let to_send = JointTrajectoryPoint {
            positions: positions.clone(),
            time_from_start: Duration { sec: c, nanosec: 0 },
            ..Default::default()
        };
        let mut native = r2r::WrappedNativeMsg::<Int32>::new();
        native.data = c;

        publisher.publish(&to_send).unwrap();
        publisher2.publish_native(&native).unwrap();
    };

    let cb2 = move |x: JointTrajectoryPoint| {
        let serialized = serde_json::to_string(&x).unwrap();
        println!("JTP serialized as: {}", serialized);
    };

    let cb3 = move |raw_c: &r2r::WrappedNativeMsg<JointTrajectoryPoint>| {
        println!("Raw c data: {:?}", raw_c.positions);
    };

    let _sub1 = node.subscribe("/hopp", Box::new(cb))?;
    let _sub2 = node.subscribe("/hej", Box::new(cb2))?;
    let _sub3 = node.subscribe_native("/hej", Box::new(cb3))?;

    // TODO: group subscriptions in a "node" struct
    //let mut subs: Vec<Box<Sub>> = vec![Box::new(sub1), Box::new(sub2), Box::new(sub3)];

    // run for 10 seconds
    let mut count = 0;
    while count < 100 {
        node.spin_once(std::time::Duration::from_millis(100));
        count += 1;
    }

    println!("All done!");

    Ok(())
}
