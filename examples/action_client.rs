use futures::executor::LocalPool;
use futures::future::FutureExt;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;

use r2r::example_interfaces::action::Fibonacci;
use std::sync::{Arc, Mutex};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let client = node.create_action_client::<Fibonacci::Action>("/fibonacci")?;
    let action_server_available = node.is_available(&client)?;

    // signal that we are done
    let done = Arc::new(Mutex::new(false));

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let task_spawner = spawner.clone();
    let task_done = done.clone();
    spawner.spawn_local(async move {
        println!("waiting for action service...");
        action_server_available
            .await
            .expect("could not await action server");
        println!("action service available.");

        let goal = Fibonacci::Goal { order: 5 };
        println!("sending goal: {:?}", goal);
        let (goal, result, feedback) = client
            .send_goal_request(goal)
            .expect("could not send goal request")
            .await
            .expect("goal rejected by server");

        println!("goal accepted: {}", goal.uuid);
        // process feedback stream in its own task
        let nested_goal = goal.clone();
        let nested_task_done = task_done.clone();
        task_spawner
            .spawn_local(feedback.for_each(move |msg| {
                let nested_task_done = nested_task_done.clone();
                let nested_goal = nested_goal.clone();
                async move {
                    println!(
                        "new feedback msg {:?} -- {:?}",
                        msg,
                        nested_goal.get_status()
                    );

                    // 50/50 that cancel the goal before it finishes.
                    if msg.sequence.len() == 4 && rand::random::<bool>() {
                        nested_goal
                            .cancel()
                            .unwrap()
                            .map(|r| {
                                println!("goal cancelled: {:?}", r);
                                // we are done.
                                *nested_task_done.lock().unwrap() = true;
                            })
                            .await;
                    }
                }
            }))
            .unwrap();

        // await result in this task
        match result.await {
            Ok((status, msg)) => {
                println!("got result {} with msg {:?}", status, msg);
                *task_done.lock().unwrap() = true;
            }
            Err(e) => println!("action failed: {:?}", e),
        }
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
        if *done.lock().unwrap() {
            break;
        }
    }

    Ok(())
}
