//! Prints the environmental information for r2r.

use r2r_common::RosMsg;

fn main() {
    println!("# r2r Information");

    println!("## Env Hash");
    println!("{}", r2r_common::get_env_hash());
    println!();

    println!("## Messages");
    println!();
    for msg in r2r_common::get_wanted_messages() {
        let RosMsg {
            module,
            prefix,
            name,
        } = msg;
        println!("- `{module}/{prefix}/{name}`");
    }
    println!();

    println!("## Cargo ROS Distro");
    println!();
    println!("```");
    r2r_common::print_cargo_ros_distro();
    println!("```");
    println!();

    println!("## Cargo Link Watches");
    println!();
    println!("```");
    r2r_common::print_cargo_watches();
    println!("```");
    println!();

    println!("## Cargo Link Search");
    println!();
    println!("```");
    r2r_common::print_cargo_link_search();
    println!("```");
    println!();
}
