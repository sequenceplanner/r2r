use std::sync::{Arc, Mutex};

use futures::future;
use futures::stream::StreamExt;

use r2r::QosProfile;
use tokio::task;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "testnode", "")?;
    let arc_node = Arc::new(Mutex::new(node));

    let an = arc_node.clone();
    task::spawn(async move { subscriber(an).await.unwrap() });

    let an = arc_node.clone();
    task::spawn(async move { publisher(an).await.unwrap() });

    let an = arc_node.clone();
    task::spawn(async move { client(an).await.unwrap() });

    let an = arc_node.clone();
    task::spawn(async move { service(an).await.unwrap() });

    let handle = tokio::task::spawn_blocking(move || loop {
        {
            arc_node
                .lock()
                .unwrap()
                .spin_once(std::time::Duration::from_millis(10));
        }
        std::thread::sleep(std::time::Duration::from_millis(100))
    });

    handle.await?;

    Ok(())
}

async fn subscriber(arc_node: Arc<Mutex<r2r::Node>>) -> Result<(), r2r::Error> {
    let sub = arc_node
        .lock()
        .unwrap()
        .subscribe::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
    sub.for_each(|msg| {
        println!("topic: new msg: {}", msg.data);
        future::ready(())
    })
    .await;
    Ok(())
}

async fn publisher(arc_node: Arc<Mutex<r2r::Node>>) -> Result<(), r2r::Error> {
    let (mut timer, publisher) = {
        // Limiting the scope when locking the arc
        let mut node = arc_node.lock().unwrap();
        let timer = node.create_wall_timer(std::time::Duration::from_secs(2))?;
        let publisher =
            node.create_publisher::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
        (timer, publisher)
    };
    for _ in 1..10 {
        timer.tick().await?;
        let msg = r2r::std_msgs::msg::String {
            data: "hello from r2r".to_string(),
        };
        publisher.publish(&msg)?;
    }
    Ok(())
}

async fn client(arc_node: Arc<Mutex<r2r::Node>>) -> Result<(), r2r::Error> {
    use r2r::example_interfaces::srv::AddTwoInts;
    let (client, mut timer, service_available) = {
        // Limiting the scope when locking the arc
        let mut node = arc_node.lock().unwrap();
        let client = node.create_client::<AddTwoInts::Service>("/add_two_ints")?;
        let timer = node.create_wall_timer(std::time::Duration::from_secs(2))?;
        let service_available = node.is_available(&client)?;
        (client, timer, service_available)
    };
    println!("waiting for service...");
    service_available.await?;
    println!("service available.");
    for i in 1..10 {
        let req = AddTwoInts::Request { a: i, b: 5 };
        if let Ok(resp) = client.request(&req).unwrap().await {
            println!("{}", resp.sum);
        }
        timer.tick().await?;
    }
    Ok(())
}

async fn service(arc_node: Arc<Mutex<r2r::Node>>) -> Result<(), r2r::Error> {
    use r2r::example_interfaces::srv::AddTwoInts;
    let mut service = {
        // Limiting the scope when locking the arc
        let mut node = arc_node.lock().unwrap();
        node.create_service::<AddTwoInts::Service>("/add_two_ints")?
    };
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
    Ok(())
}
