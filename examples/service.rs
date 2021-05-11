use r2r;
use r2r::example_interfaces::srv::AddTwoInts;

fn handle_service(request: AddTwoInts::Request) -> AddTwoInts::Response {
    println!("request: {} + {}", request.a, request.b);
    AddTwoInts::Response {
        sum: request.a + request.b
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    node.create_service::<AddTwoInts::Service>("/add_two_ints", Box::new(handle_service))?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
    }
}
