use r2r;

use r2r::example_interfaces::srv::AddTwoInts;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;
    let client = node.create_client::<AddTwoInts::Service>("/add_two_ints")?;

    // wait for service to be available
    println!("waiting for service...");
    while !node.service_available(&client)? {
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }

    println!("service available.");

    let mut c = 0;
    loop {
        let req = AddTwoInts::Request { a: 10 * c, b: c };

        let cb_req = req.clone();
        let cb = Box::new(move |r: AddTwoInts::Response| {
            println!("{} + {} = {}", cb_req.a, cb_req.b, r.sum)
        });

        client.request(&req, cb)?;

        node.spin_once(std::time::Duration::from_millis(1000));

        std::thread::sleep(std::time::Duration::from_millis(1000));
        c += 1;
    }
}
