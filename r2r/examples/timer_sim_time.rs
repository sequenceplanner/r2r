use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;

use r2r::{Clock, ClockType};
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};

async fn timer_task(
    mut t: r2r::Timer, ros_clock: Arc<Mutex<Clock>>, mut system_clock: Clock,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut iteration: i32 = 0;
    loop {
        let elapsed = t.tick().await?;

        let ros_time = ros_clock.lock().unwrap().get_now()?;
        let system_time = system_clock.get_now()?;

        println!("Timer called ({}), {}us since last call", iteration, elapsed.as_micros());
        println!(
            "\tcurrent time ros={:.3}s, system={:.3}s",
            ros_time.as_secs_f64(),
            system_time.as_secs_f64()
        );

        iteration += 1;
        if iteration == 10 {
            break;
        }
    }
    Ok(())
}

/// Publication of time can be done either using the example `sim_time_publisher`
/// or with `ros2 bag play --clock <clock_frequency> <the_bag>`
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // Simulated time can be enabled by registering parameter handler
    // and starting the program this ros param 'use_sim_time:=true'
    // ```shell
    // cargo run --features=sim-time --example=timer_sim_time -- --ros-args -p use_sim_time:=true
    // ```
    // this does not work if the parameter is changed during the runtime
    let (paramater_handler, _parameter_events) = node.make_parameter_handler()?;
    spawner.spawn_local(paramater_handler)?;

    // or simulated time can be enabled/disabled directly by calling these functions:
    // let time_source = node.get_time_source();
    // time_source.enable_sim_time(&mut node)?;
    // time_source.disable_sim_time();

    // Note: Wall timer does not use sim time
    let timer = node.create_timer(std::time::Duration::from_millis(1000))?;

    let is_done = Rc::new(RefCell::new(false));

    let task_is_done = is_done.clone();
    let ros_clock = node.get_ros_clock();
    let system_clock = Clock::create(ClockType::SystemTime)?;
    spawner.spawn_local(async move {
        match timer_task(timer, ros_clock, system_clock).await {
            Ok(()) => {
                *task_is_done.borrow_mut() = true;
                println!("exiting");
            }
            Err(e) => println!("error: {}", e),
        }
    })?;

    while !*is_done.borrow() {
        node.spin_once(std::time::Duration::from_millis(100));

        pool.run_until_stalled();
    }

    Ok(())
}
