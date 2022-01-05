#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let duration = std::time::Duration::from_millis(2500);

    use r2r::example_interfaces::srv::AddTwoInts;
    let client = node.create_client::<AddTwoInts::Service>("/add_two_ints")?;
    let mut timer = node.create_wall_timer(duration)?;
    let waiting = node.is_available(&client)?;

    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    println!("waiting for service...");
    waiting.await?;
    println!("service available.");
    for i in 1..10 {
        let req = AddTwoInts::Request { a: i, b: 5 };
        if let Ok(resp) = client.request(&req)?.await {
            println!("{}", resp.sum);
        }
        timer.tick().await?;
    }

    handle.await?;

    Ok(())
}
