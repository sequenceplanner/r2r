use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;

use r2r;

use r2r::example_interfaces::srv::AddTwoInts;

async fn requester_task(c: r2r::Client<AddTwoInts::Service>) -> Result<(), Box<dyn std::error::Error>> {
    let mut x: i64 = 0;
    loop {
        let req = AddTwoInts::Request { a: 10 * x, b: x };
        let resp = c.request(&req)?.await?;
        println!("{} + {} = {}", req.a, req.b, resp.sum);

        x+=1;
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

    // wait for service to be available
    println!("waiting for service...");
    while !node.service_available(&client)? {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }

    println!("service available.");

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    spawner.spawn_local(async move {
        match requester_task(client).await {
            Ok(()) => println!("exiting"),
            Err(e) => println!("error: {}", e),
        }})?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
