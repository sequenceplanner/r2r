use r2r;

// try to run like this
// cargo run --example parameters -- --ros-args -p param_key:=[hej,hopp] -p key2:=5.5 key2=true -r __ns:=/demo -r __node:=my_node

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "testnode", "")?;

    println!("node name: {}", node.name()?);
    println!(
        "node fully qualified name: {}",
        node.fully_qualified_name()?
    );
    println!("node namespace: {}", node.namespace()?);

    println!("node parameters");
    node.params.iter().for_each(|(k, v)| {
        println!("{} - {:?}", k, v);
    });

    Ok(())
}
