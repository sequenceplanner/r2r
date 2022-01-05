use futures::future;
use futures::stream::StreamExt;
use r2r::QosProfile;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let sub = node.subscribe::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;

    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    sub.for_each(|msg| {
        println!("topic: new msg: {}", msg.data);
        future::ready(())
    })
    .await;

    handle.await?;

    Ok(())
}
