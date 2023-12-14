use futures::stream::StreamExt;
use r2r::QosProfile;
use tokio::task;
use r2r::WrappedTypesupport;

#[tokio::test(flavor = "multi_thread")]
async fn tokio_subscribe_raw_testing() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode2", "")?;

    let mut sub_int = node.subscribe_raw("/int", "std_msgs/msg/Int32", QosProfile::default())?;

    let mut sub_array =
        node.subscribe_raw("/int_array", "std_msgs/msg/Int32MultiArray", QosProfile::default())?;

    let pub_int =
        node.create_publisher::<r2r::std_msgs::msg::Int32>("/int", QosProfile::default())?;

    // Use an array as well since its a variable sized type
    let pub_array = node.create_publisher::<r2r::std_msgs::msg::Int32MultiArray>(
        "/int_array",
        QosProfile::default(),
    )?;

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

    sub_int_handle.await?;
    sub_array_handle.await?;
    handle.join().unwrap();

    Ok(())
}



#[tokio::test(flavor = "multi_thread")]
async fn tokio_publish_raw_testing() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode2", "")?;

    let mut sub_int = node.subscribe::<r2r::std_msgs::msg::Int32>("/int", QosProfile::default())?;

    let mut sub_array =
        node.subscribe::<r2r::std_msgs::msg::Int32MultiArray>("/int_array", QosProfile::default())?;

    let pub_int = node.create_publisher_raw(
            "/int",         
            "std_msgs/msg/Int32",
            QosProfile::default()
        )?;

    // Use an array as well since its a variable sized type
    let pub_array = node.create_publisher_raw(
        "/int_array", 
        "std_msgs/msg/Int32MultiArray",
        QosProfile::default(),
    )?;

    task::spawn(async move {
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        (0..10).for_each(|i| {
            pub_int
                .publish(&r2r::std_msgs::msg::Int32 { data: i }.to_serialized_bytes().unwrap())
                .unwrap();

            pub_array
                .publish(
                    &r2r::std_msgs::msg::Int32MultiArray {
                        layout: r2r::std_msgs::msg::MultiArrayLayout::default(),
                        data: vec![i],
                    }.to_serialized_bytes().unwrap()
                )
                .unwrap();
        });
    });

    let sub_int_handle = task::spawn(async move {
        while let Some(msg) = sub_int.next().await {
            let len = msg.to_serialized_bytes().unwrap().len();

            println!("Got int msg of len {len}");
            assert_eq!(len, 8);
        }
    });

    let sub_array_handle = task::spawn(async move {
        while let Some(msg) = sub_array.next().await {
            let len = msg.to_serialized_bytes().unwrap().len();

            println!("Got array msg of len {len}");
            assert_eq!(len, 20);
        }
    });

    let handle = std::thread::spawn(move || {
        for _ in 1..=30 {
            node.spin_once(std::time::Duration::from_millis(100));
        }
    });

    sub_int_handle.await?;
    sub_array_handle.await?;
    handle.join().unwrap();

    Ok(())
}
