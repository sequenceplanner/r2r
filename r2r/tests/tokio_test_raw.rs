use futures::stream::StreamExt;
use r2r::{QosProfile, WrappedTypesupport};
use tokio::task;

const N_CONCURRENT_ROS_CONTEXT: usize = 3;
const N_TEARDOWN_CYCLES: usize = 2;

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn tokio_subscribe_raw_testing() -> Result<(), Box<dyn std::error::Error>> {
    let mut threads = futures::stream::FuturesUnordered::from_iter(
        (0..N_CONCURRENT_ROS_CONTEXT).map(|i_context| {
            tokio::spawn(async move {
                // Iterate to check for memory corruption on node setup/teardown
                for i_cycle in 0..N_TEARDOWN_CYCLES {
                    println!("tokio_subscribe_raw_testing iteration {i_cycle}");

                    let ctx = r2r::Context::create().unwrap();
                    let mut node =
                        r2r::Node::create(ctx, &format!("testnode2_{i_context}"), "").unwrap();

                    let mut sub_int = node
                        .subscribe_raw("/int", "std_msgs/msg/Int32", QosProfile::default())
                        .unwrap();

                    let mut sub_array = node
                        .subscribe_raw(
                            "/int_array",
                            "std_msgs/msg/Int32MultiArray",
                            QosProfile::default(),
                        )
                        .unwrap();

                    let pub_int = node
                        .create_publisher::<r2r::std_msgs::msg::Int32>(
                            "/int",
                            QosProfile::default(),
                        )
                        .unwrap();

                    // Use an array as well since its a variable sized type
                    let pub_array = node
                        .create_publisher::<r2r::std_msgs::msg::Int32MultiArray>(
                            "/int_array",
                            QosProfile::default(),
                        )
                        .unwrap();

                    task::spawn(async move {
                        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                        (0..10).for_each(|i| {
                            pub_int
                                .publish(&r2r::std_msgs::msg::Int32 { data: i })
                                .unwrap();

                            pub_array
                                .publish(&r2r::std_msgs::msg::Int32MultiArray {
                                    layout: r2r::std_msgs::msg::MultiArrayLayout::default(),
                                    data: vec![i],
                                })
                                .unwrap();
                        });
                    });

                    let sub_int_handle = task::spawn(async move {
                        while let Some(msg) = sub_int.next().await {
                            println!("Got int msg of len {}", msg.len());
                            assert_eq!(msg.len(), 8);
                        }
                    });

                    let sub_array_handle = task::spawn(async move {
                        while let Some(msg) = sub_array.next().await {
                            println!("Got array msg of len {}", msg.len());
                            assert_eq!(msg.len(), 20);
                        }
                    });

                    let handle = std::thread::spawn(move || {
                        for _ in 1..=30 {
                            node.spin_once(std::time::Duration::from_millis(100));
                        }
                    });

                    sub_int_handle.await.unwrap();
                    sub_array_handle.await.unwrap();
                    handle.join().unwrap();

                    println!("Going to drop tokio_subscribe_raw_testing iteration {i_cycle}");
                }
            })
        }),
    );

    while let Some(thread) = threads.next().await {
        thread.unwrap();
    }

    Ok(())
}

// Limit the number of threads to force threads to be reused
#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn tokio_publish_raw_testing() -> Result<(), Box<dyn std::error::Error>> {
    let mut threads = futures::stream::FuturesUnordered::from_iter(
        (0..N_CONCURRENT_ROS_CONTEXT).map(|i_context| {
            tokio::spawn(async move {
                // Iterate to check for memory corruption on node setup/teardown
                for i_cycle in 0..N_TEARDOWN_CYCLES {
                    println!("tokio_publish_raw_testing iteration {i_cycle}");

                    let ctx = r2r::Context::create().unwrap();
                    let mut node =
                        r2r::Node::create(ctx, &format!("testnode3_{i_context}"), "").unwrap();

                    let mut sub_int = node
                        .subscribe::<r2r::std_msgs::msg::Int32>("/int", QosProfile::default())
                        .unwrap();

                    let mut sub_array = node
                        .subscribe::<r2r::std_msgs::msg::Int32MultiArray>(
                            "/int_array",
                            QosProfile::default(),
                        )
                        .unwrap();

                    let pub_int = node
                        .create_publisher_untyped(
                            "/int",
                            "std_msgs/msg/Int32",
                            QosProfile::default(),
                        )
                        .unwrap();

                    // Use an array as well since its a variable sized type
                    let pub_array = node
                        .create_publisher_untyped(
                            "/int_array",
                            "std_msgs/msg/Int32MultiArray",
                            QosProfile::default(),
                        )
                        .unwrap();

                    task::spawn(async move {
                        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                        (0..10).for_each(|i| {
                            pub_int
                                .publish_raw(
                                    &r2r::std_msgs::msg::Int32 { data: i }
                                        .to_serialized_bytes()
                                        .unwrap(),
                                )
                                .unwrap();

                            pub_array
                                .publish_raw(
                                    &r2r::std_msgs::msg::Int32MultiArray {
                                        layout: r2r::std_msgs::msg::MultiArrayLayout::default(),
                                        data: vec![i],
                                    }
                                    .to_serialized_bytes()
                                    .unwrap(),
                                )
                                .unwrap();
                        });
                    });

                    let sub_int_handle = task::spawn(async move {
                        while let Some(msg) = sub_int.next().await {
                            // Try to check for any possible corruption
                            msg.to_serialized_bytes().unwrap();

                            println!("Got int msg with value {}", msg.data);
                            assert!(msg.data >= 0);
                            assert!(msg.data < 10);
                        }
                    });

                    let sub_array_handle = task::spawn(async move {
                        while let Some(msg) = sub_array.next().await {
                            // Try to check for any possible corruption
                            msg.to_serialized_bytes().unwrap();

                            println!("Got array msg with value {:?}", msg.data);
                            assert_eq!(msg.data.len(), 1);
                            assert!(msg.data[0] >= 0);
                            assert!(msg.data[0] < 10);
                        }
                    });

                    let handle = std::thread::spawn(move || {
                        for _ in 1..=30 {
                            node.spin_once(std::time::Duration::from_millis(100));
                        }
                    });

                    sub_int_handle.await.unwrap();
                    sub_array_handle.await.unwrap();
                    handle.join().unwrap();

                    println!("Going to drop tokio_publish_raw_testing iteration {i_cycle}");
                }
            })
        }),
    );

    while let Some(thread) = threads.next().await {
        thread.unwrap();
    }

    Ok(())
}
