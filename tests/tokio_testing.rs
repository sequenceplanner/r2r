use futures::stream::StreamExt;
use r2r::QosProfile;

use std::sync::{Arc, Mutex};
use tokio::task;

#[tokio::test(flavor = "multi_thread")]
async fn tokio_testing() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let mut s_the_no =
        node.subscribe::<r2r::std_msgs::msg::Int32>("/the_no", QosProfile::default())?;
    let mut s_new_no =
        node.subscribe::<r2r::std_msgs::msg::Int32>("/new_no", QosProfile::default())?;
    let p_the_no =
        node.create_publisher::<r2r::std_msgs::msg::Int32>("/the_no", QosProfile::default())?;
    let p_new_no =
        node.create_publisher::<r2r::std_msgs::msg::Int32>("/new_no", QosProfile::default())?;
    let state = Arc::new(Mutex::new(0));

    task::spawn(async move {
        (0..10).for_each(|i| {
            p_the_no
                .publish(&r2r::std_msgs::msg::Int32 { data: i })
                .unwrap();
        });
    });

    task::spawn(async move {
        loop {
            match s_the_no.next().await {
                Some(msg) => {
                    p_new_no
                        .publish(&r2r::std_msgs::msg::Int32 {
                            data: msg.data + 10,
                        })
                        .unwrap();
                }
                None => break,
            }
        }
    });

    let s = state.clone();
    task::spawn(async move {
        loop {
            match s_new_no.next().await {
                Some(msg) => {
                    let i = msg.data;
                    if i == 19 {
                        *s.lock().unwrap() = 19;
                    }
                }
                None => break,
            }
        }
    });

    let handle = std::thread::spawn(move || {
        for _ in 1..=30 {
            node.spin_once(std::time::Duration::from_millis(100));
            let x = state.lock().unwrap();
            if *x == 19 {
                break;
            }
        }

        *state.lock().unwrap()
    });
    let x = handle.join().unwrap();
    assert_eq!(x, 19);
    Ok(())
}
