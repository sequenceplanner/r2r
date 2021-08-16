use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::fmt::Debug;
use std::marker::PhantomData;
use std::mem::MaybeUninit;
use std::ops::{Deref, DerefMut};
use std::sync::{Arc, Mutex, Weak};
use std::time::Duration;

use futures::channel::{mpsc, oneshot};
use futures::future::FutureExt;
use futures::future::TryFutureExt;
use futures::stream::{Stream, StreamExt};
use std::future::Future;

use retain_mut::RetainMut;

// otherwise crates using r2r needs to specify the same version of uuid as
// this crate depend on, which seem like bad user experience.
pub extern crate uuid;

use actions::*;
use rcl::*;

mod error;
use error::*;

mod msg_types;
use msg_types::*;

pub use msg_types::generated_msgs::*;
pub use msg_types::WrappedNativeMsg as NativeMsg;

mod utils;
pub use utils::*;

mod subscribers;
use subscribers::*;

mod publishers;
use publishers::*;

mod services;
use services::*;

mod clients;
use clients::*;
pub use clients::{Client, UntypedClient};

mod action_clients;
use action_clients::*;

mod action_servers;
use action_servers::*;
pub use action_servers::{ActionServer, GoalRequest, ServerGoal, CancelRequest};

mod context;
pub use context::Context;

mod parameters;
use parameters::*;
pub use parameters::ParameterValue;

mod clocks;
pub use clocks::{Clock, ClockType};

mod nodes;
pub use nodes::{Node, Timer};
