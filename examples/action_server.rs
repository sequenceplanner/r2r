use futures::executor::{LocalPool, LocalSpawner};
use futures::future::{self, Either};
use futures::stream::{Stream, StreamExt};
use futures::task::LocalSpawnExt;

use r2r::example_interfaces::action::Fibonacci;
use std::sync::{Arc, Mutex};

// main goal handling routine.
async fn run_goal(
    node: Arc<Mutex<r2r::Node>>,
    g: r2r::ActionServerGoal<Fibonacci::Action>,
) -> Fibonacci::Result {
    let mut timer = node // local timer, will be dropped after this request is processed.
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
        feedback_msg
            .sequence
            .push(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1]);
        g.publish_feedback(feedback_msg.clone()).expect("fail");
        println!("Sending feedback: {:?}", feedback_msg);
        timer.tick().await.unwrap();
    }

    Fibonacci::Result {
        sequence: feedback_msg.sequence,
    }
}

async fn fibonacci_server(
    spawner: LocalSpawner,
    node: Arc<Mutex<r2r::Node>>,
    mut requests: impl Stream<Item = r2r::ActionServerGoalRequest<Fibonacci::Action>> + Unpin,
) {
    loop {
        match requests.next().await {
            Some(req) => {
                println!(
                    "Got goal request with order {}, goal id: {}",
                    req.goal.order, req.uuid
                );
                // 1/4 chance that we reject the goal for testing.
                if rand::random::<bool>() && rand::random::<bool>() {
                    println!("rejecting goal");
                    req.reject().expect("could not reject goal");
                    continue;
                }
                let (mut g, mut cancel) = req.accept().expect("could not accept goal");

                let goal_fut = spawner
                    .spawn_local_with_handle(run_goal(node.clone(), g.clone()))
                    .unwrap();

                match future::select(goal_fut, cancel.next()).await {
                    Either::Left((result, _)) => {
                        // 50/50 that we succeed or abort
                        if rand::random::<bool>() {
                            println!("goal completed!");
                            g.succeed(result).expect("could not send result");
                        } else {
                            println!("goal aborted!");
                            g.abort(result).expect("could not send result");
                        }
                    }
                    Either::Right((request, _)) => {
                        if let Some(request) = request {
                            println!("got cancel request: {}", request.uuid);
                            request.accept();
                        }
                    }
                };
            }
            None => break,
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    let ctx = r2r::Context::create()?;
    let node = Arc::new(Mutex::new(r2r::Node::create(ctx, "testnode", "")?));

    let server_requests = node
        .lock()
        .unwrap()
        .create_action_server::<Fibonacci::Action>("/fibonacci")?;

    let node_cb = node.clone();
    spawner
        .spawn_local(fibonacci_server(spawner.clone(), node_cb, server_requests))
        .unwrap();

    loop {
        node.lock()
            .unwrap()
            .spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
