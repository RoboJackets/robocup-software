//!
//! rtp is a library of the cross platform messages that are sent between the base computer
//! base station, and robots.
//! 

#![cfg_attr(feature = "nostd", no_std)]
#[cfg(feature = "nostd")]
extern crate alloc;

#[cfg(all(feature = "yellow-team", feature = "blue-team"))]
panic!("Yellow Team and Blue Team cannot be selected at the same time!!!.");

pub mod control_message;
pub use control_message::{ControlMessage, ControlMessageBuilder, CONTROL_MESSAGE_SIZE};

pub mod robot_status_message;
pub use robot_status_message::{RobotStatusMessage, RobotStatusMessageBuilder, ROBOT_STATUS_SIZE};
pub mod radio_addresses;

pub use radio_addresses::BASE_STATION_ADDRESS;
pub use radio_addresses::ROBOT_RADIO_ADDRESSES;

#[cfg(any(feature = "blue-team", not(feature = "yellow-team")))]
pub const TEAM: Team = Team::Blue;

#[cfg(feature = "yellow-team")]
pub const TEAM: Team = Team::Yellow;

// Team that Robots can be on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Team {
    // Blue Team
    Blue = 0,
    // Yellow Team
    Yellow = 1,
}

impl Into<bool> for Team {
    fn into(self) -> bool {
        match self {
            Team::Blue => false,
            Team::Yellow => true,
        }
    }
}