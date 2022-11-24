use futures::executor::LocalPool;
use futures::select;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use futures::FutureExt;

use r2r::example_interfaces::srv::AddTwoInts;

///
/// This example demonstrates how we can chain async service calls.
///
/// Run toghtether with the client example.
/// e.g. cargo run --example service
/// and in another terminal cargo run --example client
///
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let mut service = node.create_service::<AddTwoInts::Service>("/add_two_ints")?;
    let service_delayed = node.create_service::<AddTwoInts::Service>("/add_two_ints_delayed")?;
    let client = node.create_client::<AddTwoInts::Service>("/add_two_ints_delayed")?;
    let mut timer = node.create_wall_timer(std::time::Duration::from_millis(250))?;
    let mut timer2 = node.create_wall_timer(std::time::Duration::from_millis(2000))?;
    // wait for service to be available
    let service_available = node.is_available(&client)?;
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    spawner.spawn_local(async move {
        println!("waiting for delayed service...");
        service_available.await.expect("could not await service");
        println!("delayed service available.");

        loop {
            match service.next().await {
                Some(req) => {
                    println!("passing request on to delayed service");
                    let resp = client
                        .request(&req.message)
                        .expect("could not send client response")
                        .await
                        .expect("expected client response");
                    println!(
                        "responding with: {} + {} = {}",
                        req.message.a, req.message.b, resp.sum
                    );
                    req.respond(resp).expect("could not send service response");
                }
                None => break,
            }
        }
    })?;

    spawner.spawn_local(async move {
        // we need to fuse the streams for select
        let mut service_delayed = service_delayed.fuse();
        loop {
            select! {
                req = service_delayed.next() => {
                    if let Some(req) = req {
                        let resp = AddTwoInts::Response {
                            sum: req.message.a + req.message.b,
                        };
                        // wait a bit before answering...
                        let _ret = timer2.tick().await;
                        req.respond(resp).expect("could not send service response");
                    }
                },
                elapsed = timer2.tick().fuse() => {
                    if let Ok(elapsed) = elapsed {
                        println!("no request made in {}ms", elapsed.as_millis());
                    }
                }
            };
        }
    })?;

    spawner.spawn_local(async move {
        loop {
            let elapsed = timer.tick().await.expect("could not tick");
            println!(
                "doing other async work, {}ms since last call",
                elapsed.as_millis()
            );
        }
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(5));
        pool.run_until_stalled();
    }
}
