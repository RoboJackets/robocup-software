//!
//! The Robot Status Message is sent from the robots to the base station and finally to the computer
//! 

#![allow(dead_code)]

#[cfg(feature = "nostd")]
use alloc::format;

use packed_struct::prelude::*;

use crate::Team;

/// battery_voltage is a direct reading from the micrcontroller's ADC
/// and must be converted to an actual voltage, which means it should be
/// multiplied by this scale factor (TODO: Debug the scale factor)
#[allow(unused)]
pub const BATTERY_SCALE_FACTOR: f32 = 0.09884;

/// The size of a RobotStatusMessage in Bytes as a constant.
/// Note: This is tested in the tests so it can be trusted
pub const ROBOT_STATUS_SIZE: usize = 3;

/// The Robot Status Message is sent back from the robot's whenever they receive communication
/// to let software know that they are doing good.
/// 
/// Size = 3 Bytes
#[derive(PackedStruct, Clone, Copy, Debug, PartialEq, Eq)]
#[packed_struct(bit_numbering="msb0", endian="lsb")]
pub struct RobotStatusMessage {
    // Team of the RObot (0: Blue) (1: Yellow)
    #[packed_field(bits="0")]
    pub team: bool,

    // Id of the Robot
    #[packed_field(bits="1..=4")]
    pub robot_id: Integer<u8, packed_bits::Bits::<4>>,

    // True if the robot currently has ball sense
    #[packed_field(bits="5")]
    pub ball_sense_status: bool,

    // Status of the kicker (TODO: Confirm this)
    #[packed_field(bits="6")]
    pub kick_status: bool,

    // Health of the kicker
    #[packed_field(bits="7")]
    pub kick_healthy: bool,

    // Voltage measured by the ADC of the Microcontroller
    #[packed_field(bits="8..=15")]
    pub battery_voltage: Integer<u8, packed_bits::Bits::<8>>,

    // Erros experienced by the motor (TODO: Doc this better)
    #[packed_field(bits="16..=20")]
    pub motor_errors: Integer<u8, packed_bits::Bits::<5>>,

    // Status of the FPGA
    #[packed_field(bits="21")]
    pub fpga_status: bool,

    // Unused bits so the encoded deltas line up against a byte boundary
    #[packed_field(bits="22.=23")]
    unused: Integer<u8, packed_bits::Bits::<2>>,
}

pub struct RobotStatusMessageBuilder {
    pub team: Option<Team>,
    pub robot_id: Option<u8>,
    pub ball_sense_status: Option<bool>,
    pub kick_status: Option<bool>,
    pub kick_healthy: Option<bool>,
    pub battery_voltage: Option<u8>,
    pub motor_errors: Option<u8>,
    pub fpga_status: Option<bool>,
}

impl RobotStatusMessageBuilder {
    pub fn new() -> Self {
        Self {
            team: None,
            robot_id: None,
            ball_sense_status: None,
            kick_status: None,
            kick_healthy: None,
            battery_voltage: None,
            motor_errors: None,
            fpga_status: None,
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

    pub fn ball_sense_status(mut self, ball_sense_status: bool) -> Self {
        self.ball_sense_status = Some(ball_sense_status);
        self
    }

    pub fn kick_status(mut self, kick_status: bool) -> Self {
        self.kick_status = Some(kick_status);
        self
    }

    pub fn kick_healthy(mut self, kick_healthy: bool) -> Self {
        self.kick_healthy = Some(kick_healthy);
        self
    }

    pub fn battery_voltage(mut self, battery_voltage: u8) -> Self {
        self.battery_voltage = Some(battery_voltage);
        self
    }

    pub fn motor_errors(mut self, motor_errors: u8) -> Self {
        self.motor_errors = Some(motor_errors);
        self
    }

    pub fn fpga_status(mut self, fpga_status: bool) -> Self {
        self.fpga_status = Some(fpga_status);
        self
    }

    pub fn build(self) -> RobotStatusMessage {
        let team = match self.team {
            Some(team) => if team == Team::Blue { false } else { true },
            None => false,
        };

        let robot_id = match self.robot_id {
            Some(robot_id) => robot_id,
            None => 0,
        };

        let ball_sense_status = match self.ball_sense_status {
            Some(ball_sense_status) => ball_sense_status,
            None => false,
        };

        let kick_status = match self.kick_status {
            Some(kick_status) => kick_status,
            None => false,
        };

        let kick_healthy = match self.kick_healthy {
            Some(kick_healthy) => kick_healthy,
            None => false,
        };

        let battery_voltage = match self.battery_voltage {
            Some(battery_voltage) => battery_voltage,
            None => 0,
        };

        let motor_errors = match self.motor_errors {
            Some(motor_errors) => motor_errors,
            None => 0,
        };

        let fpga_status = match self.fpga_status {
            Some(fpga_status) => fpga_status,
            None => false,
        };

        RobotStatusMessage {
            team,
            robot_id: robot_id.into(),
            ball_sense_status,
            kick_status,
            kick_healthy,
            battery_voltage: battery_voltage.into(),
            motor_errors: motor_errors.into(),
            fpga_status: fpga_status.into(),
            unused: 0.into(),
        }
    }
}

#[cfg(feature = "std")]
mod tests {
    use super::*;

    /// Test that the RobotStatusMessageBuilder uses the correct default fields when
    /// they are not provided.
    #[test]
    fn test_empty_robot_status_message_builder() {
        let robot_status = RobotStatusMessageBuilder::new().build();

        let expected = RobotStatusMessage {
            team: false,
            robot_id: 0.into(),
            ball_sense_status: false,
            kick_status: false,
            kick_healthy: false,
            battery_voltage: 0.into(),
            motor_errors: 0.into(),
            fpga_status: false,
            unused: 0.into(),
        };

        assert_eq!(expected, robot_status);
    }

    /// Test that the RobotStatusMessageBuilder uses the filled in fields to create
    /// a RobotStatusMessage.
    #[test]
    fn test_complete_robot_status_message_builder() {
        let robot_status = RobotStatusMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(1)
            .ball_sense_status(true)
            .kick_status(true)
            .kick_healthy(true)
            .battery_voltage(10)
            .motor_errors(2)
            .fpga_status(true)
            .build();

        let expected: RobotStatusMessage = RobotStatusMessage {
            team: true,
            robot_id: 1.into(),
            ball_sense_status: true,
            kick_status: true,
            kick_healthy: true,
            battery_voltage: 10.into(),
            motor_errors: 2.into(),
            fpga_status: true,
            unused: 0.into(),
        };

        assert_eq!(expected, robot_status);
    }

    /// The Robot Status for
    /// RobotStatusMessage {
    ///     team: Yellow (true),
    ///     robot_id: 1,
    ///     ball_sense_status: true,
    ///     kick_status: true,
    ///     kick_healthy: false,
    ///     battery_voltage: 10,
    ///     motor_errors: 0,
    ///     fpga_status: true,
    /// }
    /// 
    /// is as follows:
    ///     1_0001_1_1_0 | 00001010 | 00000_1_00
    ///     ^   ^  ^ ^ ^       ^        ^   ^  ^
    ///     |   |  | | |       |        |   |  |
    /// team-   |  | | |       |        |   |  |
    /// robot_id-  | | |       |        |   |  |
    /// ball_sense-- | |       |        |   |  |
    /// kick_status--- |       |        |   |  |
    /// kick_healthy----       |        |   |  |
    /// battery_voltage---------        |   |  |
    /// motor_errors---------------------   |  |
    /// fpga_status--------------------------  |
    /// unused----------------------------------
    /// 
    #[test]
    fn test_pack() {
        let robot_status = RobotStatusMessageBuilder::new()
            .team(Team::Yellow)
            .robot_id(1)
            .ball_sense_status(true)
            .kick_status(true)
            .battery_voltage(10)
            .motor_errors(0)
            .fpga_status(true)
            .build();

        let packed_data = match robot_status.pack() {
            Ok(bytes) => bytes,
            Err(err) => panic!("Unable to pack robot status: {:?}", err),
        };

        assert_eq!(packed_data.len(), ROBOT_STATUS_SIZE);
        assert_eq!(packed_data[0], 0b1_0001_1_1_0);
        assert_eq!(packed_data[1], 0b00001010);
        assert_eq!(packed_data[2], 0b00000_1_00);
    }

    /// The Robot Status for the slice:
    ///     1_0001_1_1_0 | 00001010 | 00000_1_00
    ///     ^   ^  ^ ^ ^       ^        ^   ^  ^
    ///     |   |  | | |       |        |   |  |
    /// team-   |  | | |       |        |   |  |
    /// robot_id-  | | |       |        |   |  |
    /// ball_sense-- | |       |        |   |  |
    /// kick_status--- |       |        |   |  |
    /// kick_healthy----       |        |   |  |
    /// battery_voltage---------        |   |  |
    /// motor_errors---------------------   |  |
    /// fpga_status--------------------------  |
    /// unused----------------------------------
    /// 
    /// is as follows:
    /// RobotStatusMessage {
    ///     team: Yellow (true),
    ///     robot_id: 1,
    ///     ball_sense_status: true,
    ///     kick_status: true,
    ///     kick_healthy: false,
    ///     battery_voltage: 10,
    ///     fpga_status: true,
    /// }
    #[test]
    fn test_unpack() {
        let status_slice: [u8; 3] = [0b1_0001_1_1_0, 0b00001010, 0b00000_1_00];
        let robot_status = match RobotStatusMessage::unpack_from_slice(&status_slice[..]) {
            Ok(status) => status,
            Err(err) => panic!("Unable to Unpack Robot Status Message: {:?}", err),
        };

        let expected = RobotStatusMessage {
            team: true,
            robot_id: 1.into(),
            ball_sense_status: true,
            kick_status: true,
            kick_healthy: false,
            battery_voltage: 10.into(),
            fpga_status: true,
            motor_errors: 0.into(),
            unused: 0.into(),
        };

        assert_eq!(expected ,robot_status);
    }
}