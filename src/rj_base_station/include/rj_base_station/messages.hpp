/**
 * @file messages.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Radio messages sent and received when talking to the robots
 * @version 0.1
 * @date 2026-03-08
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once
#pragma GCC diagnostic ignored "-Wconversion"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>

namespace rtp {

/**
 * @brief The radio addresses for the blue and yellow team for both radios
 */
static constexpr std::array<std::array<std::array<uint8_t, 5>, 2>, 2> kBaseStationAddresses = {
    {{{
         // Blue Team
         {{0xE7, 0xE7, 0xE7, 0xE7, 0xE7}},  // Radio 1
         {{0xB6, 0xB6, 0xB6, 0xB6, 0xB6}}   // Radio 2
     }},
     {{
         // Yellow Team
         {{0xA4, 0xA4, 0xA4, 0xA4, 0xA4}},  // Radio 1
         {{0x63, 0x63, 0x63, 0x63, 0x63}}   // Radio 2
     }}}};

/**
 * @brief The radio addresses for the robots for both teams. The first index is the team (0 for
 * blue, 1 for yellow), the second index is the robot number (0-5), and the third index is the byte
 * of the address (0-4).
 *
 */
static constexpr std::array<std::array<std::array<uint8_t, 5>, 6>, 2> kRobotAddresses = {
    {{{
         // Blue Team
         {{0xC3, 0xC3, 0xC3, 0xC3, 0xC1}},  // Robot 1
         {{0xC3, 0xC3, 0xC3, 0xC3, 0xC2}},  // Robot 2
         {{0xC3, 0xC3, 0xC3, 0xC3, 0xC3}},  // Robot 3
         {{0xC3, 0xC3, 0xC3, 0xC3, 0xC4}},  // Robot 4
         {{0xC3, 0xC3, 0xC3, 0xC3, 0xC5}},  // Robot 5
         {{0xC3, 0xC3, 0xC3, 0xC3, 0xC6}}   // Robot 6
     }},
     {{
         // Yellow Team
         {{0xD5, 0xD5, 0xD5, 0xD5, 0xD1}},  // Robot 1
         {{0xD5, 0xD5, 0xD5, 0xD5, 0xD2}},  // Robot 2
         {{0xD5, 0xD5, 0xD5, 0xD5, 0xD3}},  // Robot 3
         {{0xD5, 0xD5, 0xD5, 0xD5, 0xD4}},  // Robot 4
         {{0xD5, 0xD5, 0xD5, 0xD5, 0xD5}},  // Robot 5
         {{0xD5, 0xD5, 0xD5, 0xD5, 0xD6}}   // Robot 6
     }}}};

/**
 * @brief Special enum to dictate the mode the robot should be in
 *
 */
enum ControlMode {
    DEFAULT = 0,
    IMU_TEST = 1,
    RECEIVE_BENCHMARK = 2,
    SEND_BENCHMARK = 3,
    PROGRAM_KICK_ON_BREAKBEAM = 4,
    PROGRAM_KICKER = 5,
    KICKER_TEST = 6,
    FPGA_TEST = 7,
};

/**
 * @brief How the kicker should be triggered
 *
 */
enum TriggerMode {
    StandDown = 0,
    Immediate = 1,
    OnBreakBeam = 2,
};

/**
 * @brief How should the kicker kick the ball
 *
 */
enum ShootMode {
    Kick = 0,
    Chip = 1,
};

struct ControlMessage {
    // The size of a control message
    static constexpr size_t kSize = 10;
    // The scale factor to and from message velocities
    static constexpr double kVelocityScaleFactor = 1000.0;

    static ControlMessage from_ros(
        unsigned int robot_id, bool blue_team,
        const std::shared_ptr<rj_msgs::msg::MotionSetpoint>&
            motion_setpoint,  // NOLINT(bugprone-easily-swappable-parameters)
        const std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint>& manipulator_setpoint) {
        TriggerMode trigger_mode;
        if (manipulator_setpoint->trigger_mode == 0) {
            trigger_mode = TriggerMode::StandDown;
        } else if (manipulator_setpoint->trigger_mode == 1) {
            trigger_mode = TriggerMode::Immediate;
        } else {
            trigger_mode = TriggerMode::OnBreakBeam;
        }
        return {
            .team = blue_team ? 0 : 1,
            .robot_id = robot_id,
            .shoot_mode =
                manipulator_setpoint->kick_speed > 0.0 ? ShootMode::Kick : ShootMode::Chip,
            .trigger_mode = trigger_mode,
            .body_x = static_cast<int16_t>(motion_setpoint->velocity_x_mps * kVelocityScaleFactor),
            .body_y = static_cast<int16_t>(motion_setpoint->velocity_y_mps * kVelocityScaleFactor),
            .body_w =
                static_cast<int16_t>(motion_setpoint->velocity_z_radps * kVelocityScaleFactor),
            .dribbler_speed = static_cast<int8_t>(manipulator_setpoint->dribbler_speed * 100),
            .kick_strength = static_cast<uint8_t>(manipulator_setpoint->kick_speed * 100),
            .role = 0,
            .unused = 0};
    }

    // 0 for blue team 1 for yellow team
    unsigned team : 1;
    // Id of the robot
    unsigned robot_id : 4;
    // The shoot mode
    unsigned shoot_mode : 1;
    // The trigger mode
    unsigned trigger_mode : 2;
    // The x velocity In body frame)
    int16_t body_x;
    // The y velocity (In body frame)
    int16_t body_y;
    // The w velocity (In body frame)
    int16_t body_w;
    // The dribbler speed
    int8_t dribbler_speed;
    // The kick strength
    uint8_t kick_strength;
    // Role of the robot (currently unused)
    unsigned role : 2;
    // Unused bytes
    unsigned unused : 6;

} __attribute__((packed));

struct RobotStatusMessage {
    // The size of a RobotStatusMessage in bytes
    static constexpr size_t kSize = 3;

    /**
     * @brief Convert an rtp RobotStatus message to a ros message
     *
     * @return rj_msgs::msg::RobotStatus
     */
    [[nodiscard]] rj_msgs::msg::RobotStatus to_ros() const {
        rj_msgs::msg::RobotStatus status;
        status.robot_id = robot_id;
        status.kicker_healthy = kicker_healthy;
        status.has_ball_sense = ball_sense;
        status.battery_voltage = battery_voltage;
        std::array<bool, 5> m_errors = {false};
        for (size_t i = 0; i < 5; i++) {
            m_errors[i] = (motor_errors & (1 << i)) > 0;
        }
        status.motor_errors = m_errors;
        return status;
    }

    // 0 for blue team 1 for yellow team
    unsigned team : 1;
    // Id of the robot
    unsigned robot_id : 4;
    // Does the robot have ball sense?
    unsigned ball_sense : 1;
    // What is the kicker status?
    unsigned kicker_status : 1;
    // Is the kicker healthy?
    unsigned kicker_healthy : 1;
    // Battery voltage measured by the ADC of hte microcontroller
    uint8_t battery_voltage;
    // unused data
    unsigned unused : 2;
    // Is the FPGA working? (deprecated)
    unsigned fpga_status : 1;
    // Motor errors (deprecated)
    unsigned motor_errors : 5;
} __attribute__((packed));

}  // namespace rtp
