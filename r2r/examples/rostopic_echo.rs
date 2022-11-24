use futures::executor::LocalPool;
use futures::future;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r::QosProfile;

use std::collections::HashMap;
use std::env;
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "echo", "")?;

    let args: Vec<String> = env::args().collect();
    let topic = args.get(1).expect("provide a topic!");

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // run for a while to populate the topic list (note blocking...)
    let mut count = 0;
    let mut nt = HashMap::new();
    while count < 50 {
        thread::sleep(std::time::Duration::from_millis(10));
        nt = node.get_topic_names_and_types()?;
        if nt.contains_key(topic) {
            break;
        }
        count += 1;
    }

    let type_name = nt.get(topic).and_then(|types| types.get(0));
    let type_name = match type_name {
        Some(tn) => tn,
        None => {
            eprintln!("Could not determine the type for the passed topic");
            return Ok(());
        }
    };

    println!("topic: {}, type: {}", topic, type_name);

    // create echo topic
    let echo = &format!("{}_echo", topic);
    let echo_pub = node.create_publisher_untyped(echo, type_name, QosProfile::default())?;

    let sub = node.subscribe_untyped(topic, type_name, QosProfile::default())?;
    spawner.spawn_local(async move {
        sub.for_each(|msg| {
            match msg {
                Ok(msg) => {
                    let s = serde_json::to_string_pretty(&msg).unwrap();
                    println!("{}\n---\n", &s);
                    echo_pub.publish(msg).unwrap();
                }
                Err(err) => {
                    println!("Could not parse msg. {}", err);
                }
            }
            future::ready(())
        })
        .await
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
