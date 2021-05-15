use std::sync::mpsc;
use std::thread;

use r2r;
use r2r::std_msgs;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let publisher = node.create_publisher::<std_msgs::msg::String>("/hello")?;
    let pubint = node.create_publisher::<std_msgs::msg::Int32>("/count")?;

    let (tx, rx) = mpsc::channel::<String>();
    thread::spawn(move || loop {
        if let Ok(msg) = rx.recv() {
            let deserialized: std_msgs::msg::String = serde_json::from_str(&msg).unwrap();
            println!(
                "received: {}, deserialized ros msg = {:?}",
                msg, deserialized
            );
        } else {
            println!("stopping");
            break;
        }
    });

    let mut c = 0;
    let cb = move |x: std_msgs::msg::String| {
        let to_send = format!("at count {} got: {}", c, x.data);
        c = c + 1;
        let serialized = serde_json::to_string(&x).unwrap();
        tx.send(serialized).unwrap(); // pass msg on to other thread for printing
        let to_send = std_msgs::msg::String { data: to_send };
        publisher.publish(&to_send).unwrap();
        let to_send = std_msgs::msg::Int32 { data: c };
        pubint.publish(&to_send).unwrap();
    };

    let _ws2 = node.subscribe("/hi", Box::new(cb))?;

    // run for 10 seconds
    let mut count = 0;
    while count < 100 {
        node.spin_once(std::time::Duration::from_millis(100));
        count += 1;
    }

    Ok(())
}
