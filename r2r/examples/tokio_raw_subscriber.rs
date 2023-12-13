use futures::future;
use futures::stream::StreamExt;
use r2r::QosProfile;
use serde::{Deserialize, Serialize};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let p = node.create_publisher::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
    let sub = node.subscribe_raw("/topic", "std_msgs/msg/String", QosProfile::default())?;

    let pub_task = tokio::task::spawn(async move {
        for x in 5..50 {
            // Send a string with varying length.
            let _ = p.publish(&r2r::std_msgs::msg::String {
                data: format!("Hello{:>width$}", "World", width = x),
            });
            let _ = tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
    });

    tokio::task::spawn_blocking(move || {
        for _ in 0..500 {
            node.spin_once(std::time::Duration::from_millis(10));
        }
    });

    // Demonstrate that we can deserialize the raw bytes into this
    // rust struct using the cdr crate.
    #[derive(Deserialize, Serialize, PartialEq, Debug)]
    struct OurOwnStdString {
        data: String, // the field name can be anything...
    }
    sub.for_each(|msg| {
        println!("got raw bytes of length {}.", msg.len());

        if let Ok(data) = cdr::deserialize::<OurOwnStdString>(&msg) {
            println!("contents: {:?}", data);
        } else {
            println!("Warning: cannot deserialize data.");
        }

        future::ready(())
    })
    .await;

    pub_task.await?;

    Ok(())
}
