//! Easy to use, runtime-agnostic async rust bindings for ROS2.
//! ---
//!
//! Minimal bindings for ROS2 that do *not* require hooking in to the
//! ROS2 build infrastructure -- `cargo build` is all you
//! need. Convenience Rust types are created by calling into the c
//! introspection libraries. This circumvents the ROS2 .msg/.idl
//! pipeline by relying on already generated C code. By default, the
//! behavior is to build bindings to the RCL and all message types
//! that can be found in the currently sourced ros environment.
//!
//! What works?
//!---
//!- Up to date with ROS2 ~Dashing~ ~Eloquent~ Foxy Galactic
//!- Building Rust types
//!- Publish/subscribe
//!- Services
//!- Actions
//!- Rudimentary parameter handling
//!
//! ---
//!
//! After having sourced ROS2 (see README for more details), you can
//! try the following example:
//!
//! ``` rust
//!use futures::executor::LocalPool;
//!use futures::future;
//!use futures::stream::StreamExt;
//!use futures::task::LocalSpawnExt;
//!use r2r::QosProfile;
//!
//!fn main() -> Result<(), Box<dyn std::error::Error>> {
//!    let ctx = r2r::Context::create()?;
//!    let mut node = r2r::Node::create(ctx, "node", "namespace")?;
//!    let subscriber = node.subscribe::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
//!    let publisher = node.create_publisher::<r2r::std_msgs::msg::String>("/topic", QosProfile::default())?;
//!    let mut timer = node.create_wall_timer(std::time::Duration::from_millis(1000))?;
//!
//!    // Set up a simple task executor.
//!    let mut pool = LocalPool::new();
//!    let spawner = pool.spawner();
//!
//!    // Run the subscriber in one task, printing the messages
//!    spawner.spawn_local(async move {
//!        subscriber.for_each(|msg| {
//!            println!("got new msg: {}", msg.data);
//!            future::ready(())
//!        }).await
//!    })?;
//!
//!    // Run the publisher in another task
//!    spawner.spawn_local(async move {
//!        let mut counter = 0;
//!        loop {
//!            let _elapsed = timer.tick().await.unwrap();
//!            let msg = r2r::std_msgs::msg::String { data: format!("Hello, world! ({})", counter) };
//!            publisher.publish(&msg).unwrap();
//!            counter += 1;
//!        }
//!    })?;
//!
//!    // Main loop spins ros.
//!    loop {
//!        node.spin_once(std::time::Duration::from_millis(100));
//!        pool.run_until_stalled();
//!    }
//!}
//! ```

// otherwise crates using r2r needs to specify the same version of uuid as
// this crate depend on, which seem like bad user experience.
pub extern crate uuid;

mod error;
pub use error::{Error, Result};

mod msg_types;
pub use msg_types::generated_msgs::*;
pub use msg_types::WrappedNativeMsg as NativeMsg;

mod utils;
pub use utils::*;

mod subscribers;

mod publishers;
pub use publishers::{Publisher, PublisherUntyped};

mod services;
pub use services::ServiceRequest;

mod clients;
pub use clients::{Client, ClientUntyped};

mod action_common;
pub use action_common::GoalStatus;

mod action_clients;
pub use action_clients::{ActionClient, ActionClientGoal};

mod action_clients_untyped;
pub use action_clients_untyped::{ActionClientGoalUntyped, ActionClientUntyped};

mod action_servers;
pub use action_servers::{ActionServerCancelRequest, ActionServerGoal, ActionServerGoalRequest};

mod context;
pub use context::Context;

mod parameters;
pub use parameters::ParameterValue;

mod clocks;
pub use clocks::{Clock, ClockType};

mod nodes;
pub use nodes::{Node, Timer};

pub mod qos;
pub use qos::QosProfile;
