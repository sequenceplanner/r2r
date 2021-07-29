use r2r;

// try to run like this
// cargo run --example parameters -- --ros-args -p key1:=[hello,world] -p key2:=5.5 -r __ns:=/demo -r __node:=my_node
// then run
// ros2 param set /demo/my_node key2 false

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "to_be_replaced", "to_be_replaced")?;

    println!("node name: {}", node.name()?);
    println!(
        "node fully qualified name: {}",
        node.fully_qualified_name()?
    );
    println!("node namespace: {}", node.namespace()?);

    let mut i = 0;
    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        if i % 20 == 0 { // every 2 seconds print all parameters
            println!("node parameters");
            node.params.lock().unwrap().iter().for_each(|(k, v)| {
                println!("{} - {:?}", k, v);
            });
        }
        i+=1;
        if i > 1000 {
            break;
        }
    }

    Ok(())
}
