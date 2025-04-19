use std::time::Duration;

use futures::{executor::LocalPool, task::LocalSpawnExt};

use r2r::{std_msgs::msg, std_srvs::srv, QosProfile};


/// This example demonstrates creation of a service,
/// subscriber and timers with their callback execution traced.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // The traced callback is supplied directly to `subscribe_traced`
    // and `create_service_traced`functions.
    let subscriber_future =
        node.subscribe_traced("/print", QosProfile::default(), |msg: msg::String| {
            println!("Received message: '{}'", msg.data);
        })?;
    spawner.spawn_local(subscriber_future)?;

    let mut value = false;
    let service_future = node.create_service_traced::<srv::SetBool::Service, _>(
        "/set_value",
        QosProfile::default(),
        move |req| {
            if value == req.message.data {
                req.respond(srv::SetBool::Response {
                    success: false,
                    message: format!("Value is already {value}."),
                })
                .expect("could not send service response");
            } else {
                value = req.message.data;
                req.respond(srv::SetBool::Response {
                    success: true,
                    message: format!("Value set to {value}."),
                })
                .expect("could not send service response");
            }
        },
    )?;
    spawner.spawn_local(service_future)?;

    let mut counter = 0;
    let wall_timer_future =
        node.create_wall_timer(Duration::from_millis(500))?
            .on_tick(move |_| {
                counter += 1;
                println!("Wall timer tick: {counter}");
            });
    spawner.spawn_local(wall_timer_future)?;

    let ros_timer_future = node.create_timer(Duration::from_secs(1))?.on_tick(|diff| {
        println!("ROS timer tick. Time elapsed since last tick: {diff:?}");
    });
    spawner.spawn_local(ros_timer_future)?;

    loop {
        node.spin_once(std::time::Duration::from_millis(5));
        pool.run_until_stalled();
    }
}
