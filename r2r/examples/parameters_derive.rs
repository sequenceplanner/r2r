use futures::{executor::LocalPool, prelude::*, task::LocalSpawnExt};
use r2r::RosParams;
use std::sync::{Arc, Mutex};

// try to run like this
// cargo run --example parameters_derive -- --ros-args -p par1:=5.1 -p nested.par4:=42 -r __ns:=/demo -r __node:=my_node
// then run
// ros2 param get /demo/my_node nested.par4 # should return 42
// ros2 param set /demo/my_node nested.par4 43
// ros2 param set /demo/my_node nested.par4 xxx # fails due to invalid type
// ros2 param set /demo/my_node nested.nested2.par5 999 # fails with conversion error
// ros2 param dump /demo/my_node
// Prints:
//   /demo/my_node:
//     ros__parameters:
//       nested:
//         nested2:
//           par5: 0
//         par3: initial value
//         par4: 43
//       par1: 5.1
//       par2: 0
//
// ros2 param describe /demo/my_node par1 nested.nested2.par5
// Prints:
//   Parameter name: par1
//     Type: double
//     Description: Parameter description
//     Constraints:
//   Parameter name: nested.nested2.par5
//     Type: integer
//     Description: Small parameter
//     Constraints:

// Error handling:
// cargo run --example parameters_derive -- --ros-args -p nested.par4:=xxx

// Explore how is RosParams derived by running:
// cargo expand --example=parameters_derive

#[derive(RosParams, Default, Debug)]
struct Params {
    /// Parameter description
    par1: f64,
    /// Dummy parameter [m/s]
    par2: i32,
    nested: NestedParams,
}

#[derive(RosParams, Default, Debug)]
struct NestedParams {
    par3: String,
    par4: u16,
    nested2: NestedParams2,
}

#[derive(RosParams, Default, Debug)]
struct NestedParams2 {
    /// Small parameter
    par5: i8,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Ros version: {}", r2r::ROS_DISTRO);

    // set up executor
    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    // set up ros node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "to_be_replaced", "to_be_replaced")?;

    // create our parameters and set default values
    let params = Arc::new(Mutex::new({
        let mut p = Params::default();
        p.nested.par3 = "initial value".into();
        p
    }));

    // make a parameter handler (once per node).
    // the parameter handler is optional, only spawn one if you need it.
    let (paramater_handler, parameter_events) =
        node.make_derived_parameter_handler(params.clone())?;
    // run parameter handler on your executor.
    spawner.spawn_local(paramater_handler)?;

    println!("node name: {}", node.name()?);
    println!("node fully qualified name: {}", node.fully_qualified_name()?);
    println!("node namespace: {}", node.namespace()?);

    // parameter event stream. just print them
    let params_clone = params.clone();
    spawner.spawn_local(async move {
        parameter_events
            .for_each(|_| {
                println!("event: {:#?}", params_clone.lock().unwrap());
                future::ready(())
            })
            .await
    })?;

    // print all params every 5 seconds.
    let mut timer = node.create_wall_timer(std::time::Duration::from_secs(5))?;
    spawner.spawn_local(async move {
        loop {
            println!("timer: {:#?}", params.lock().unwrap());
            let _elapsed = timer.tick().await.expect("could not tick");
        }
    })?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
    }
}
