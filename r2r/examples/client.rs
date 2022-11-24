use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use futures::Future;

use std::io::Write;

use r2r::example_interfaces::srv::AddTwoInts;

async fn requester_task(
    node_available: impl Future<Output = r2r::Result<()>>,
    c: r2r::Client<AddTwoInts::Service>,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut x: i64 = 0;
    println!("waiting for service...");
    node_available.await?;
    println!("service available.");
    loop {
        let req = AddTwoInts::Request { a: 10 * x, b: x };
        print!("{} + {} = ", req.a, req.b);
        std::io::stdout().flush()?;
        let resp = c.request(&req)?.await?;
        println!("{}", resp.sum);

        x += 1;
        if x == 10 {
            break;
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let client = node.create_client::<AddTwoInts::Service>("/add_two_ints")?;

    let service_available = node.is_available(&client)?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    spawner.spawn_local(async move {
        match requester_task(service_available, client).await {
            Ok(()) => println!("done."),
            Err(e) => println!("error: {}", e),
        }
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
