use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let node = r2r::Node::create(ctx, "testnode", "")?;

    loop {
        let nt = node.get_topic_names_and_types()?;
        let mut keys: Vec<&String> = nt.keys().collect();
        keys.sort();
        println!("---------------------");
        for k in keys {
            println!("{}: {:?}", k, nt[k]);
        }
        thread::sleep(Duration::from_millis(500));
    }
}
