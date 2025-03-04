//! 
//! The Control Message is sent to robots over radio to inform them of what actions to take.
//! 

#![allow(dead_code)]

#[cfg(feature = "nostd")]
use alloc::format;

#[cfg(feature = "nostd")]
use nalgebra::base::*;

use packed_struct::prelude::*;

use crate::Team;

/// The body{X, Y, W} are multiplied (upon sending) by the VELOCITY_SCALE_FACTOR and divided
/// (upon receiving) to preserve at least 3 decimals of floating point precision.
pub const VELOCITY_SCALE_FACTOR: f32 = 1000.0;

/// The size of a ControlMessage in Bytes as a constant.
/// Note: This is tested in the tests so it can be trusted
pub const CONTROL_MESSAGE_SIZE: usize = 10;

/// The Trigger Mode Kicking
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriggerMode {
    StandDown = 0,
    Immediate = 1,
    OnBreakBeam = 2,
}

impl Into<u8> for TriggerMode {
    fn into(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShootMode {
    Kick = 0,
    Chip = 1,
}

impl Into<bool> for ShootMode {
    fn into(self) -> bool {
        match self {
            ShootMode::Kick => false,
            ShootMode::Chip => true,
        }
    }
}

/// The Control Message is Sent from the Base Station to the Robots.
/// 
/// Size = 80 Bits = 10 Bytes
#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering="msb0", endian="lsb")]
pub struct ControlMessage {
    // Team of the Robot (0: Blue) (1: Yellow)
    #[packed_field(bits="0")]
    pub team: bool,

    // Id of the Robot
    #[packed_field(bits="1..=4")]
    pub robot_id: Integer<u8, packed_bits::Bits::<4>>,

    // Mode of kicking for the robot
    // 0 -> Kick
    // 1 -> Chip
    #[packed_field(bits="5")]
    pub shoot_mode: bool,

    // Trigger Mode for the Robot (TODO: Finish Docs)
    #[packed_field(bits="6..=7")]
    pub trigger_mode: Integer<u8, packed_bits::Bits::<2>>,

    // X Coordinate of the Robot's Body Frame (multiplied by VELOCITY_SCALE_FACTOR
    // and truncated)
    #[packed_field(bits="8..=23")]
    pub body_x: Integer<i16, packed_bits::Bits::<16>>,

    // Y Coordinate of the Robot's Body Frame (multiplied by VELOCITY_SCALE_FACTOR
    // and truncated)
    #[packed_field(bits="24..=39")]
    pub body_y: Integer<i16, packed_bits::Bits::<16>>,

    // W Coordinate of the Robot's Body Frame (multiplied by VELOCITY_SCALE_FACTOR
    // and truncated))
    #[packed_field(bits="40..=55")]
    pub body_w: Integer<i16, packed_bits::Bits::<16>>,

    // Speed of the dribbler (TODO: Determine Units)
    #[packed_field(bits="56..=63")]
    pub dribbler_speed: Integer<i8, packed_bits::Bits::<8>>,

    // Strength of the kicker on kick (TODO: Determine Units)
    #[packed_field(bits="64..=71")]
    pub kick_strength: Integer<u8, packed_bits::Bits::<8>>,

    // Role of This Robot (TODO: Finish Docs)
    #[packed_field(bits="72..=73")]
    pub role: Integer<u8, packed_bits::Bits<2>>,

    // Unused Bits to make this struct an even byte length
    #[packed_field(bits="74..=79")]
    unused: Integer<u8, packed_bits::Bits::<6>>,
}

impl ControlMessage {
    #![cfg(feature = "nostd")]
    pub fn get_velocity(&self) -> Vector3<f32> {
        Vector3::new(
            (*self.body_x as f32) / VELOCITY_SCALE_FACTOR,
            (*self.body_y as f32) / VELOCITY_SCALE_FACTOR,
            (*self.body_w as f32) / VELOCITY_SCALE_FACTOR,
        )
    }
}

pub struct ControlMessageBuilder {
    pub team: Option<Team>,
    pub robot_id: Option<u8>,
    pub shoot_mode: Option<ShootMode>,
    pub trigger_mode: Option<TriggerMode>,
    pub body_x: Option<i16>,
    pub body_y: Option<i16>,
    pub body_w: Option<i16>,
    pub dribbler_speed: Option<i8>,
    pub kick_strength: Option<u8>,
    pub role: Option<u8>,
}

impl ControlMessageBuilder {
    pub fn new() -> Self {
        Self {
            team: None,
            robot_id: None,
            shoot_mode: None,
            trigger_mode: None,
            body_x: None,
            body_y: None,
            body_w: None,
            dribbler_speed: None,
            kick_strength: None,
            role: None,
        }
    }

    pub fn team(mut self, team: Team) -> Self {
        self.team = Some(team);
        self
    }

    pub fn robot_id(mut self, robot_id: u8) -> Self {
        self.robot_id = Some(robot_id);
        self
    }

    pub fn shoot_mode(mut self, shoot_mode: ShootMode) -> Self {
        self.shoot_mode = Some(shoot_mode);
        self
    }

    pub fn trigger_mode(mut self, trigger_mode: TriggerMode) -> Self {
        self.trigger_mode = Some(trigger_mode);
        self
    }

    pub fn body_x(mut self, body_x: f32) -> Self {
        self.body_x = Some((body_x * VELOCITY_SCALE_FACTOR) as i16);
        self
    }

    pub fn body_y(mut self, body_y: f32) -> Self {
        self.body_y = Some((body_y * VELOCITY_SCALE_FACTOR) as i16);
        self
    }

    pub fn body_w(mut self, body_w: f32) -> Self {
        self.body_w = Some((body_w * VELOCITY_SCALE_FACTOR) as i16);
        self
    }

    pub fn dribbler_speed(mut self, dribbler_speed: i8) -> Self {
        self.dribbler_speed = Some(dribbler_speed);
        self
    }

    pub fn kick_strength(mut self, kick_strength: u8) -> Self {
        self.kick_strength = Some(kick_strength);
        self
    }

    pub fn role(mut self, role: u8) -> Self {
        self.role = Some(role);
        self
    }

    pub fn build(self) -> ControlMessage {
        let team = match self.team {
            Some(team) => if team == Team::Blue { false } else { true },
            None => false,
        };

        let robot_id = match self.robot_id {
            Some(robot_id) => robot_id,
            None => 0,
        };

        let shoot_mode = match self.shoot_mode {
            Some(shoot_mode) => shoot_mode.into(),
            None => false,
        };

        let trigger_mode = match self.trigger_mode {
            Some(trigger_mode) => trigger_mode.into(),
            None => 0,
        };

        let body_x = match self.body_x {
            Some(body_x) => body_x,
            None => 0,
        };

        let body_y = match self.body_y {
            Some(body_y) => body_y,
            None => 0,
        };

        let body_w = match self.body_w {
            Some(body_w) => body_w,
            None => 0,
        };

        let dribbler_speed = match self.dribbler_speed {
            Some(dribbler_speed) => dribbler_speed,
            None => 0,
        };

        let kick_strength = match self.kick_strength {
            Some(kick_strength) => kick_strength,
            None => 0,
        };

        let role = match self.role {
            Some(role) => role,
            None => 0,
        };

        ControlMessage {
            team,
            robot_id: robot_id.into(),
            shoot_mode,
            trigger_mode: trigger_mode.into(),
            body_x: body_x.into(),
            body_y: body_y.into(),
            body_w: body_w.into(),
            dribbler_speed: dribbler_speed.into(),
            kick_strength: kick_strength.into(),
            role: role.into(),
            unused: 0.into(),
        }
    }
}

#[cfg(feature = "std")]
mod tests {
    use super::*;

    /// Test that ControlMessageBuilder uses the correct default fields when
    /// they are not provided.
    #[test]
    fn test_empty_control_message_builder() {
        let control_message = ControlMessageBuilder::new().build();

        let expected = ControlMessage {
            team: false,
            robot_id: 0.into(),
            shoot_mode: false,
            trigger_mode: 0.into(),
            body_x: 0.into(),
            body_y: 0.into(),
            body_w: 0.into(),
            dribbler_speed: 0.into(),
            kick_strength: 0.into(),
            role: 0.into(),
            unused: 0.into(),
        };

        assert_eq!(expected, control_message);
    }

    /// Test that the ControlMessageBuilder uses the correct fields when they
    /// are provided
    #[test]
    fn test_complete_control_message_builder() {
        let control_message = ControlMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(3)
            .shoot_mode(ShootMode::Chip)
            .trigger_mode(TriggerMode::OnBreakBeam)
            .body_x(20.0)
            .body_y(10.0)
            .body_w(45.0)
            .dribbler_speed(-5)
            .kick_strength(3)
            .role(1)
            .build();

        let expected = ControlMessage {
            team: true,
            robot_id: 3.into(),
            shoot_mode: true,
            trigger_mode: 2.into(),
            body_x: 20_000.into(),
            body_y: 10_000.into(),
            body_w: 32_767.into(),
            dribbler_speed: (-5).into(),
            kick_strength: 3.into(),
            role: 1.into(),
            unused: 0.into(),
        };

        assert_eq!(expected, control_message);
    }

    /// The Control Message for
    /// ControlMessage {
    ///     team: Yellow (false),
    ///     robot_id: 3,
    ///     shoot_mode: Chip (1),
    ///     trigger_mode: OnBreakBeam (2),
    ///     body_x: 20.0 (20_000),
    ///     body_y: 10.0 (10_000),
    ///     body_w: 45.0 (32_767),
    ///     dribbler_speed: -5,
    ///     role: 1,
    /// }
    /// 
    /// is as follows:
    ///     1_0011_1_10 | 00100000 | 01001110 | 00010000 | 00100111 | ...
    ///     ^   ^  ^  ^       ^          ^         ^          ^
    ///     |   |  |  |       |          |         |          |
    /// team-   |  |  |       |          |         |          |
    /// robot_id-  |  |       |          |         |          |
    /// shoot_mode--  |       |          |         |          |
    /// trigger_mode---       |          |         |          |
    /// body_x (lsb)-----------          |         |          |
    /// body_x (msb)----------------------         |          |
    /// body_y (lsb)--------------------------------          |
    /// body_y (msb)-------------------------------------------
    /// 
    ///     11111111 | 01111111 | 11111011 | 00000011 | 01_000000
    ///         ^          ^          ^          ^       ^    ^
    ///         |          |          |          |       |    |
    /// body_w (lsb)       |          |          |       |    |
    /// body_w (msb)--------          |          |       |    |
    /// dribbler_speed (2s Comp)-------          |       |    |
    /// kick_strength-----------------------------       |    |
    /// role----------------------------------------------    |
    /// unused-------------------------------------------------
    #[test]
    fn test_pack() {
        let control_message = ControlMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(3)
            .shoot_mode(ShootMode::Chip)
            .trigger_mode(TriggerMode::OnBreakBeam)
            .body_x(20.0)
            .body_y(10.0)
            .body_w(45.0)
            .dribbler_speed(-5)
            .kick_strength(3)
            .role(1)
            .build();

        let packed_data = match control_message.pack() {
            Ok(bytes) => bytes,
            Err(err) => panic!("Unable to pack control message: {:?}", err),
        };

        assert_eq!(packed_data.len(), CONTROL_MESSAGE_SIZE);
        assert_eq!(packed_data[0], 0b1_0011_1_10);
        assert_eq!(packed_data[1], 0b00100000);
        assert_eq!(packed_data[2], 0b01001110);
        assert_eq!(packed_data[3], 0b00010000);
        assert_eq!(packed_data[4], 0b00100111);
        assert_eq!(packed_data[5], 0b11111111);
        assert_eq!(packed_data[6], 0b01111111);
        assert_eq!(packed_data[7], 0b11111011);
        assert_eq!(packed_data[8], 0b00000011);
        assert_eq!(packed_data[9], 0b01_000000);
    }

    /// The Control Message from:
    ///     1_0011_1_10 | 00100000 | 01001110 | 00010000 | 00100111 | ...
    ///     ^   ^  ^  ^       ^          ^         ^          ^
    ///     |   |  |  |       |          |         |          |
    /// team-   |  |  |       |          |         |          |
    /// robot_id-  |  |       |          |         |          |
    /// shoot_mode--  |       |          |         |          |
    /// trigger_mode---       |          |         |          |
    /// body_x (lsb)-----------          |         |          |
    /// body_x (msb)----------------------         |          |
    /// body_y (lsb)--------------------------------          |
    /// body_y (msb)-------------------------------------------
    /// 
    ///     11111111 | 01111111 | 11111011 | 00000011 | 01_000000
    ///         ^          ^          ^          ^       ^    ^
    ///         |          |          |          |       |    |
    /// body_w (lsb)       |          |          |       |    |
    /// body_w (msb)--------          |          |       |    |
    /// dribbler_speed (2s Comp)-------          |       |    |
    /// kick_strength-----------------------------       |    |
    /// role----------------------------------------------    |
    /// unused-------------------------------------------------
    /// 
    /// is as follows:
    /// ControlMessage {
    ///     team: Yellow (false),
    ///     robot_id: 3,
    ///     shoot_mode: Chip (1),
    ///     trigger_mode: OnBreakBeam (2),
    ///     body_x: 20.0 (20_000),
    ///     body_y: 10.0 (10_000),
    ///     body_w: 45.0 (32_767),
    ///     dribbler_speed: -5,
    ///     role: 1,
    /// }
    #[test]
    fn test_unpack() {
        let data = [0b1_0011_1_10, 0b00100000, 0b01001110, 0b00010000, 0b00100111,
                               0b11111111, 0b01111111, 0b11111011, 0b00000011, 0b01_000000];

        let control_message = match ControlMessage::unpack_from_slice(&data[..]) {
            Ok(control_message) => control_message,
            Err(err) => panic!("Unable to Unpack Control Message: {:?}", err),
        };

        let expected = ControlMessage {
            team: true,
            robot_id: 3.into(),
            shoot_mode: true,
            trigger_mode: 2.into(),
            body_x: 20_000.into(),
            body_y: 10_000.into(),
            body_w: 32_767.into(),
            dribbler_speed: (-5).into(),
            kick_strength: 3.into(),
            role: 1.into(),
            unused: 0.into(),
        };

        assert_eq!(expected, control_message);
    }
}