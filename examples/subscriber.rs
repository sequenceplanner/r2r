use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let mut sub = node.subscribe::<r2r::std_msgs::msg::String>("/topic")?;
    let p = node.create_publisher::<r2r::std_msgs::msg::String>("/topic2")?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // task that every other time forwards message to topic2
    spawner.spawn_local(async move {
        let mut x: i32 = 0;
        loop {
            match sub.next().await {
                Some(msg) => {
                    if x % 2 == 0 {
                        p.publish(&r2r::std_msgs::msg::String {
                            data: format!("({}): new msg: {}", x, msg.data),
                        })
                        .unwrap();
                    }
                }
                None => break,
            }
            x += 1;
        }
    })?;

    // for sub2 we just print the data
    let sub2 = node.subscribe::<r2r::std_msgs::msg::String>("/topic2")?;
    spawner.spawn_local(async move {
        sub2.for_each(|msg| {
            println!("topic2: new msg: {}", msg.data);
            future::ready(())
        })
        .await
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
