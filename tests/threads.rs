use std::thread;
use std::time::Duration;
use failure::Error;

use r2r;

#[test]
// Let's create and drop a lot of node and publishers for a while to see that we can cope.
fn doesnt_crash() -> Result<(), Error> {
    // a global shared context.
    let ctx = r2r::Context::create()?;

    for c in 0..10 {
        let mut ths = Vec::new();
        for i in 0..30 {
            // create concurrent nodes that max out the cpu
            let ctx = ctx.clone();
            ths.push(thread::spawn(move || {
                let mut node = r2r::Node::create(ctx, &format!("testnode{}", i), "").unwrap();

                // each with 10 publishers
                for _j in 0..10 {
                    let p = node
                        .create_publisher::<r2r::std_msgs::msg::String>(&format!("/r2r{}", i))
                        .unwrap();
                    let to_send = r2r::std_msgs::msg::String {
                        data: format!("[node{}]: {}", i, c),
                    };

                    // move publisher to its own thread and publish as fast as we can
                    thread::spawn(move || loop {
                        let res = p.publish(&to_send);
                        thread::sleep(Duration::from_millis(1));
                        match res {
                            Ok(_) => (),
                            Err(_) => {
                                // println!("publisher died, quitting thread.");
                                break;
                            }
                        }
                    });
                }

                // spin to simulate some load
                for _j in 0..100 {
                    node.spin_once(Duration::from_millis(10));
                }

                // println!("all done {}-{}", c, i);
            }));
        }

        for t in ths {
            t.join().unwrap();
        }
        // println!("all threads done {}", c);
    }

    Ok(())
}
