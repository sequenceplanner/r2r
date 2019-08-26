use std::sync::mpsc;
use std::thread;
use std::time::Duration;

use r2r::*;

#[test]
// Let's create and drop a lot of node and publishers for a while to see that we can cope.
fn doesnt_crash() -> Result<(), ()> {
    // a global shared context.
    let ctx = Context::create()?;

    for c in 0..10 {
        let mut ths = Vec::new();
        for i in 0..10 {
            let ctx = ctx.clone();
            ths.push(thread::spawn(move || {
                let mut node = Node::create(ctx, &format!("testnode{}", i), "").unwrap();
                let p = node
                    .create_publisher::<std_msgs::msg::String>(&format!("/r2r{}", i))
                    .unwrap();

                let to_send = std_msgs::msg::String {
                    data: format!("[node{}]: {}", i, c),
                };

                // move publisher to its own thread and publish as fast as we can
                thread::spawn(move || {
                    loop {
                        let res = p.publish(&to_send);
                        match res {
                            Ok(_) => (),
                            Err(_) => {
                                // println!("publisher died, quitting thread.");
                                break;
                            }
                        }
                    }
                });

                // wait for 1 sec before dropping
                std::thread::sleep(Duration::from_millis(1000));
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
