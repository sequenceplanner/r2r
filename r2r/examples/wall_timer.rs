use futures::executor::LocalPool;
use futures::task::LocalSpawnExt;

use std::cell::RefCell;
use std::rc::Rc;

async fn timer_task(mut t: r2r::Timer) -> Result<(), Box<dyn std::error::Error>> {
    let mut x: i32 = 0;
    loop {
        let elapsed = t.tick().await?;
        println!(
            "timer called ({}), {}us since last call",
            x,
            elapsed.as_micros()
        );

        x += 1;
        if x == 10 {
            break;
        }
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let timer = node.create_wall_timer(std::time::Duration::from_millis(1000))?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let is_done = Rc::new(RefCell::new(false));

    let task_is_done = is_done.clone();
    spawner.spawn_local(async move {
        match timer_task(timer).await {
            Ok(()) => {
                *task_is_done.borrow_mut() = true;
                println!("exiting");
            }
            Err(e) => println!("error: {}", e),
        }
    })?;

    while !*is_done.borrow() {
        node.spin_once(std::time::Duration::from_millis(100));

        pool.run_until_stalled();
    }

    Ok(())
}
