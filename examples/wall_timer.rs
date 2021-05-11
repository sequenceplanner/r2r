use r2r;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let mut x = 0;
    let cb = move |elapsed: std::time::Duration| {
        println!("timer called ({}), {}us since last call", x, elapsed.as_micros());
        x+=1;
    };
    node.create_wall_timer(std::time::Duration::from_millis(2000), Box::new(cb))?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }
}
