use futures::executor::LocalPool;
use futures::future::FutureExt;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use std::sync::{Arc,Mutex};
use r2r;
use r2r::example_interfaces::action::Fibonacci;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let client = Arc::new(Mutex::new(node.create_action_client::<Fibonacci::Action>("/fibonacci")?));

    // signal that we are done
    let done = Arc::new(Mutex::new(false));

    println!("waiting for action service...");
    while !node.action_server_available(&client.lock().unwrap())? {
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    println!("action service available.");

    let goal = Fibonacci::Goal { order: 10 };
    println!("sending goal: {:?}", goal);
    let goal_fut = client.lock().unwrap().send_goal_request(goal)?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let task_spawner = spawner.clone();
    let task_done = done.clone();
    spawner.spawn_local(async move {
        let mut goal = goal_fut.await.unwrap(); // assume success

        println!("goal accepted: {}", goal.uuid);
        // process feedback stream in its own task
        let nested_goal = goal.clone();
        let nested_task_done = task_done.clone();
        task_spawner
            .spawn_local(goal.get_feedback().unwrap().for_each(move |msg| {
                let nested_task_done = nested_task_done.clone();
                let nested_goal = nested_goal.clone();
                async move {
                    println!("new feedback msg {:?} -- {:?}", msg, nested_goal.get_status());

                    // cancel the goal before it finishes. (comment out to complete the goal)
                    if msg.sequence.len() == 8 {
                        nested_goal.cancel().unwrap().
                            map(|r| {
                                println!("goal cancelled: {:?}", r);
                                // we are done.
                                *nested_task_done.lock().unwrap() = true;
                            }).await;
                    }
                }})).unwrap();

        // await result in this task
        let result = goal.get_result().unwrap().await;
        match result {
            Ok(msg) => {
                println!("got result {:?}", msg);
                *task_done.lock().unwrap() = true;
            },
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
