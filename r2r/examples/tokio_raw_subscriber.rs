use futures::{future, stream::StreamExt};
use r2r::{QosProfile, WrappedTypesupport};
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

    sub.for_each(|msg| {
        println!("got raw bytes with size: {}. deserialize...", msg.len());

        // We can use the ROS typesupport to perform deserialization.
        let ros_str = r2r::std_msgs::msg::String::from_serialized_bytes(&msg);

        // Demonstrate that it is possible to also deserialize the raw
        // bytes into a rust struct using the `cdr` crate.
        #[derive(Deserialize, Serialize, PartialEq, Debug)]
        struct OurOwnStdString {
            data: String, // the field name can be anything...
        }
        let cdr_str = cdr::deserialize::<OurOwnStdString>(&msg);

        match (ros_str, cdr_str) {
            (Ok(s1), Ok(s2)) => {
                assert!(s1.data == s2.data);
                println!("... using ros: {:?}", s1);
                println!("... using cdr: {:?}", s2);
            }
            _ => println!("Error: cannot deserialize data."),
        }
        future::ready(())
    })
    .await;

    pub_task.await?;

    Ok(())
}
