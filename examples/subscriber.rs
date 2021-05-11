use r2r;
use std::sync::mpsc;
use std::thread;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;

    let th = {
        let mut node = r2r::Node::create(ctx, "testnode", "")?;

        let (tx, rx) = mpsc::channel::<String>();

        let p = node
            .create_publisher::<r2r::std_msgs::msg::String>("/hej")
            .unwrap();

        let th = thread::spawn(move || loop {
            println!("thread looping");
            let des = if let Ok(msg) = rx.recv() {
                let deserialized: r2r::std_msgs::msg::String = serde_json::from_str(&msg).unwrap();
                println!(
                    "received: {}, deserialized ros msg = {:#?}",
                    msg, deserialized
                );
                deserialized
            } else {
                break;
            };

            if let Err(_) = p.publish(&des) {
                break;
            }
        });

        let tx1 = tx.clone();
        let cb = move |x: r2r::std_msgs::msg::String| {
            let serialized = serde_json::to_string(&x).unwrap();
            tx1.send(serialized).unwrap(); // pass msg on to other thread for printing
        };

        let cb2 = move |x: &r2r::WrappedNativeMsg<r2r::std_msgs::msg::String>| {
            // use native data!
            let s = x.data.to_str();
            println!("native ros msg: {}", s);
        };

        let _subref = node.subscribe("/hopp", Box::new(cb))?;
        let _subref = node.subscribe_native("/hoppe", Box::new(cb2))?;

        // run for 10 seconds
        let mut count = 0;
        while count < 100 {
            node.spin_once(std::time::Duration::from_millis(100));
            count += 1;
        }
        th
    };

    println!("dropped node");

    th.join().unwrap();

    println!("all done");

    Ok(())
}
