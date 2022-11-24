use futures::future;
use futures::stream::StreamExt;
use r2r::QosProfile;

use std::sync::{Arc, Mutex};
use tokio::task;

#[derive(Debug, Default)]
struct SharedState {
    pub state: i32,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let mut sub = node.subscribe::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
    let p =
        node.create_publisher::<r2r::std_msgs::msg::String>("/topic2", QosProfile::default())?;
    let state = Arc::new(Mutex::new(SharedState::default()));

    // task that every other time forwards message to topic2
    let state_t1 = state.clone();
    task::spawn(async move {
        let mut x: i32 = 0;
        loop {
            match sub.next().await {
                Some(msg) => {
                    if x % 2 == 0 {
                        p.publish(&r2r::std_msgs::msg::String {
                            data: format!("({}): new msg: {}", x, msg.data),
                        })
                        .unwrap();
                    } else {
                        // update shared state
                        state_t1.lock().unwrap().state = x;
                    }
                }
                None => break,
            }
            x += 1;
        }
    });

    // for sub2 we just print the data
    let sub2 = node.subscribe::<r2r::std_msgs::msg::String>("/topic2", QosProfile::default())?;
    task::spawn(async move {
        sub2.for_each(|msg| {
            println!("topic2: new msg: {}", msg.data);
            future::ready(())
        })
        .await
    });

    let mut timer = node
        .create_wall_timer(std::time::Duration::from_millis(2500))
        .unwrap();
    let state_t2 = state;
    task::spawn(async move {
        loop {
            let time_passed = timer.tick().await.unwrap();
            let x = state_t2.lock().unwrap().state;
            println!(
                "timer event. time passed: {}. shared state is {}",
                time_passed.as_micros(),
                x
            );
        }
    });

    // here we spin the node in its own thread (but we could just busy wait in this thread)
    let handle = std::thread::spawn(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });
    handle.join().unwrap();

    Ok(())
}
