use futures::stream::StreamExt;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    use r2r::example_interfaces::srv::AddTwoInts;
    let mut service = node.create_service::<AddTwoInts::Service>("/add_two_ints")?;

    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    loop {
        match service.next().await {
            Some(req) => {
                let resp = AddTwoInts::Response {
                    sum: req.message.a + req.message.b,
                };
                req.respond(resp).expect("could not send service response");
            }
            None => break,
        }
    }

    handle.await?;

    Ok(())
}
