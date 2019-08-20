use r2r::*;
use builtin_interfaces::msg::Duration;
use trajectory_msgs::msg::*;
use std_msgs::msg::Int32;


fn main() -> Result<(), ()> {
    let mut ctx = rcl_create_context()?;
    let mut node = rcl_create_node(&mut ctx, "testnode", "")?;
    let publisher = rcl_create_publisher::<JointTrajectoryPoint>(&mut node, "/hej")?;
    let publisher2 = rcl_create_publisher::<Int32>(&mut node, "/native_count")?;

    let mut c = 0;
    let cb = move |x:std_msgs::msg::String| {
        println!("at count {} got: {}", c, x.data);
        c = c + 1;
        let positions: Vec<f64> = vec!(94.2 * c as f64);
        let to_send = JointTrajectoryPoint {
            positions,
            time_from_start : Duration { sec: c, nanosec: 0 },
            ..Default::default()
        };
        let mut native = WrappedNativeMsg::<Int32>::new();
        native.data = c;

        publish(&publisher, &to_send).unwrap();
        publish_native(&publisher2, &native).unwrap();
    };

    let cb2 = move |x:JointTrajectoryPoint| {
        let serialized = serde_json::to_string(&x).unwrap();
        println!("JTP serialized as: {}", serialized);
    };    

    let cb3 = move |raw_c:&WrappedNativeMsg<JointTrajectoryPoint>| {
        println!("Raw c data: {:?}", raw_c.positions);
    };

    let sub1 = rcl_create_subscription(&mut node, "/hopp", Box::new(cb))?;
    let sub2 = rcl_create_subscription(&mut node, "/hej", Box::new(cb2))?;
    let sub3 = rcl_create_subscription_native(&mut node, "/hej", Box::new(cb3))?;

    // TODO: group subscriptions in a "node" struct
    let mut subs: Vec<Box<Sub>> = vec![Box::new(sub1), Box::new(sub2), Box::new(sub3)];

    // run for 10 seconds
    let mut count = 0;
    while count < 100 {
        let timeout = 100 * 1000 * 1000; // 0.1 sec
        take_subs(&mut ctx, &mut subs, timeout)?;
        count = count + 1;
    }


    // TODO: crashes here. maybe because pub and sub are not cleaned up
    rcl_destroy_node(&mut node);
    rcl_destroy_ctx(&mut ctx);

    Ok(())

}

