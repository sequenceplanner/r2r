use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use futures::Future;

async fn requester_task(
    node_available: impl Future<Output = r2r::Result<()>>,
    c: r2r::ClientUntyped,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut x: i64 = 0;
    println!("waiting for service...");
    node_available.await?;
    println!("service available.");
    loop {
        let json = format!("{{ \"a\": {}, \"b\": {} }}", 10 * x, x);
        let req = serde_json::from_str(&json).unwrap();
        let resp = c.request(req)?.await?;
        println!("{}", resp.expect("deserialization error"));

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
    let client =
        node.create_client_untyped("/add_two_ints", "example_interfaces/srv/AddTwoInts")?;

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
