use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;
use r2r;
use r2r::example_interfaces::action::Fibonacci;
use r2r::ServerGoal;
use std::sync::{Arc, Mutex};

// note: cannot be blocking.
fn accept_goal_cb(uuid: &uuid::Uuid, goal: &Fibonacci::Goal) -> bool {
    println!(
        "Got goal request with order {}, goal id: {}",
        goal.order, uuid
    );
    // reject high orders
    goal.order < 100
}

// note: cannot be blocking.
fn accept_cancel_cb(goal: &ServerGoal<Fibonacci::Action>) -> bool {
    println!("Got request to cancel {}", goal.uuid);
    // always accept cancel requests
    true
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let task_spawner = spawner.clone();

    let ctx = r2r::Context::create()?;
    let node = Arc::new(Mutex::new(r2r::Node::create(ctx, "testnode", "")?));

    // signal that we are done
    let done = Arc::new(Mutex::new(false));

    let node_cb = node.clone();
    let done_cb = done.clone();
    let handle_goal_cb = move |mut g: ServerGoal<Fibonacci::Action>| {
        // note that we cannot create the timer here, since we are
        // called during spin_once which menas whoever is spinning holds the mutex.
        // instead we just set up and immediately start a task.
        // also we cannot block which is why we spawn the task
        let node_cb = node_cb.clone();
        let done_cb = done_cb.clone();
        task_spawner
            .spawn_local(async move {
                let mut timer = node_cb
                    .lock()
                    .unwrap()
                    .create_wall_timer(std::time::Duration::from_millis(1000))
                    .expect("could not create timer");
                let mut feedback_msg = Fibonacci::Feedback {
                    sequence: vec![0, 1],
                };
                g.publish_feedback(feedback_msg.clone()).expect("fail");
                let order = g.goal.order as usize;
                for i in 1..order {
                    if g.is_cancelling() {
                        println!("Goal cancelled. quitting");
                        let result_msg = Fibonacci::Result {
                            sequence: feedback_msg.sequence,
                        };
                        g.cancel(result_msg).expect("could not send cancel request");
                        // signal stopping of the node
                        *done_cb.lock().unwrap() = true;
                        return;
                    }
                    feedback_msg
                        .sequence
                        .push(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1]);
                    g.publish_feedback(feedback_msg.clone()).expect("fail");
                    println!("Sending feedback: {:?}", feedback_msg);

                    timer.tick().await.unwrap();
                }
                let result_msg = Fibonacci::Result {
                    sequence: feedback_msg.sequence,
                };
                g.succeed(result_msg).expect("could not set result");
                // signal stopping of the node
                *done_cb.lock().unwrap() = true;
            })
            .expect("could not spawn task");
    };

    let _server = node
        .lock()
        .unwrap()
        .create_action_server::<Fibonacci::Action>(
            "/fibonacci",
            Box::new(accept_goal_cb),
            Box::new(accept_cancel_cb),
            Box::new(handle_goal_cb),
        )?;
    loop {
        node.lock()
            .unwrap()
            .spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
        if *done.lock().unwrap() {
            break;
        }
    }

    Ok(())
}
