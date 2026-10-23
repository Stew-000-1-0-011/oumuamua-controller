//! In-process demo robot: a simulated robot and bridge server on 127.0.0.1 that speaks the
//! same UDP/MessagePack protocol as the ROS bridge (docs/protocol.md), so the app can be tried
//! with no robot. Behaviour follows `ros/.../fake_robot.py`, simplified.

mod server;
mod shell;
mod sim;

pub use server::DemoRobot;
