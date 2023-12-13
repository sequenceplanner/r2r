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
        while let Some(msg) = s_the_no.next().await {
            p_new_no
                .publish(&r2r::std_msgs::msg::Int32 {
                    data: msg.data + 10,
                })
                .unwrap();
        }
    });

    let s = state.clone();
    task::spawn(async move {
        while let Some(msg) = s_new_no.next().await {
            let i = msg.data;
            if i == 19 {
                *s.lock().unwrap() = 19;
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


#[tokio::test(flavor = "multi_thread")]
async fn tokio_subscribe_raw_testing() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    
    let mut sub_int =
        node.subscribe_raw("/int", "std_msgs/msg/Int32", QosProfile::default())?;

    let mut sub_array =
        node.subscribe_raw("/int_array", "std_msgs/msg/Int32MultiArray", QosProfile::default())?;

    let pub_int =
        node.create_publisher::<r2r::std_msgs::msg::Int32>("/int", QosProfile::default())?;

    // Use an array as well since its a variable sized type
    let pub_array =
        node.create_publisher::<r2r::std_msgs::msg::Int32MultiArray>("/int_array", QosProfile::default())?;


    task::spawn(async move {
        (0..10).for_each(|i| {
            pub_int
                .publish(&r2r::std_msgs::msg::Int32 { data: i })
                .unwrap();

            pub_array.publish(&r2r::std_msgs::msg::Int32MultiArray { 
                layout: r2r::std_msgs::msg::MultiArrayLayout::default(),
                data: vec![i] 
            })
            .unwrap();
        });
    });

     
    let sub_int_handle = task::spawn(async move {
        while let Some(msg) = sub_int.next().await {
            println!("Got int msg of len {}", msg.len());

            // assert_eq!(msg.len(), 4);
            // TODO is there padding or something?
            assert_eq!(msg.len(), 8);
            
            // assert_eq!(msg,)
  
        }

        panic!("int msg finished");
    });

    let sub_array_handle = task::spawn(async move {
        while let Some(msg) = sub_array.next().await {

            println!("Got array msg of len {}", msg.len());
            // assert_eq!(msg.data, )
  
        }

        panic!("array msg finished");
    });

    let handle = std::thread::spawn(move || {
        for _ in 1..=30 {
            node.spin_once(std::time::Duration::from_millis(100));

        }

    });

    handle.join().unwrap();

    // This means something panicked ..
    assert!(!sub_int_handle.is_finished());
    assert!(!sub_array_handle.is_finished());

    Ok(())
}
