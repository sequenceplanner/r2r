use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r;
use r2r::example_interfaces::action::Fibonacci;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let client = node.create_action_client::<Fibonacci::Action>("/fibonacci")?;

    println!("waiting for action service...");
    while !node.action_server_available(&client)? {
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    println!("action service available.");

    let goal = Fibonacci::Goal { order: 10 };
    println!("sending goal: {:?}", goal);
    let goal_fut = client.send_goal_request(goal)?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let task_spawner = spawner.clone();
    spawner.spawn_local(async move {
        let goal = goal_fut.await.unwrap(); // assume success

        // process feedback stream in its own task
        task_spawner
            .spawn_local(goal.feedback.for_each(|msg| {
                println!("new feedback msg {:?}", msg);
                future::ready(())
            }))
            .unwrap();

        // await result in this task
        let result = goal.result.await;
        match result {
            Ok(msg) => println!("got result {:?}", msg),
            Err(e) => println!("action failed: {:?}", e),
        }
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
