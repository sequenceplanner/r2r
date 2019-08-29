use std::sync::mpsc;
use std::thread;
use failure::Error;

use r2r::*;

fn main() -> Result<(), Error> {
    let ctx = Context::create()?;
    let mut node = Node::create(ctx, "testnode", "")?;

    let publisher = node.create_publisher::<std_msgs::msg::String>("/hej")?;
    let pubint = node.create_publisher::<std_msgs::msg::Int32>("/count")?;

    let (tx, rx) = mpsc::channel::<String>();
    thread::spawn(move || loop {
        let msg = rx.recv().unwrap();
        let deserialized: std_msgs::msg::String = serde_json::from_str(&msg).unwrap();
        println!(
            "received: {}, deserialized ros msg = {:?}",
            msg, deserialized
        );
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

    let _ws2 = node.subscribe("/hopp", Box::new(cb))?;

    // run for 10 seconds
    let mut count = 0;
    while count < 100 {
        node.spin_once(std::time::Duration::from_millis(100));
        count += 1;
    }

    Ok(())
}
