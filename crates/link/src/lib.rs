//! Phone-side link to the ROS bridge: reliable channel, UDP client thread, beacon discovery.

pub mod client;
pub mod discovery;
pub mod reliable;

pub use client::{Client, ControlInput, LinkEvent};
pub use discovery::Discovery;
