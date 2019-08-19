use std::thread;
use std::sync::mpsc;

use r2r::*;

fn main() -> Result<(), ()> {
    let mut ctx = rcl_create_context()?;
    let mut node = rcl_create_node("qqq", "", &mut ctx)?;

    let publisher = rcl_create_publisher::<std_msgs::msg::String>(&mut node, "/hej")?;
    let pubint = rcl_create_publisher::<std_msgs::msg::Int32>(&mut node, "/count")?;

    let (tx, rx) = mpsc::channel::<std_msgs::msg::String>();
    thread::spawn(move|| {
        loop {
            let msg = rx.recv().unwrap();
            println!("other thread received: {}", msg.data);
        }
    });

    let mut c = 0;
    let cb = move |x:std_msgs::msg::String| {
        let to_send = format!("at count {} got: {}", c, x.data);
        c = c + 1;
        tx.send(x.clone()).unwrap(); // pass msg on to other thread for printing
        let to_send = std_msgs::msg::String { data: to_send };
        publish(&publisher, &to_send).unwrap();
        let to_send = std_msgs::msg::Int32 { data: c };
        publish(&pubint, &to_send).unwrap();
    };

    let ws2 = rcl_create_subscription(&mut node, "/hopp", Box::new(cb))?;

    // TODO: group subscriptions in a "node" struct
    let mut subst: Vec<Box<Sub>> = vec![Box::new(ws2)];

    // run for 10 seconds
    let mut count = 0;
    while count < 100 {
        let timeout = 100 * 1000 * 1000; // 0.1 sec
        rcl_take_subst(&mut ctx, &mut subst, timeout)?;
        count = count + 1;
    }


    // TODO: crashes here. maybe because pub and sub are not cleaned up
    rcl_destroy_node(&mut node);
    rcl_destroy_ctx(&mut ctx);

    Ok(())

}

