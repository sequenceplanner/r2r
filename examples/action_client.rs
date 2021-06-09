use r2r;
use r2r::example_interfaces::action::Fibonacci;
use std::cell::RefCell;
use std::rc::Rc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let client = node.create_action_client::<Fibonacci::Action>("/fibonacci")?;

    println!("waiting for action service...");
    while !node.action_server_available(&client)? {
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    println!("action service available.");

    let goal = Fibonacci::Goal { order: 10 };
    let goal_accepted = Rc::new(RefCell::new(None));
    let cb_ga = goal_accepted.clone();
    let cb = Box::new(move |r: Fibonacci::SendGoal::Response| {
        println!("got response {:?}", r);
        *cb_ga.borrow_mut() = Some(r.accepted);
    });

    let feedback_cb = Box::new(move |fb: Fibonacci::Feedback| {
        println!("got feedback {:?}", fb);
    });

    let result_cb = Box::new(move |r: Fibonacci::Result| {
        println!("final result {:?}", r);
    });

    println!("sending goal: {:?}", goal);
    client.send_goal_request(goal, cb, feedback_cb, result_cb)?;

    let mut c = 0;
    loop {
        node.spin_once(std::time::Duration::from_millis(1000));
        std::thread::sleep(std::time::Duration::from_millis(1000));
        c += 1;
        if c > 100 {
            println!("shutdown");
            break;
        }
    }

    Ok(())
}
