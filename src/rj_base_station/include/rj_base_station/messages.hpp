/**
 * @file messages.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The various radio messages for talking to the robots
 * @version 0.1
 * @date 2025-12-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once
#pragma GCC diagnostic ignored "-Wconversion"

#include <cstdint>
#include <string>
#include <vector>
#include <array>

#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>

namespace rtp {

/**
 * @brief The radio addresses of the yellow and blue base stations
 * 
 */
static constexpr std::array<std::array<uint8_t, 5>, 2> kBaseStationAddresses = {{
    {{0xD0, 0xC3, 0xC3, 0xC3, 0xC3}}, // yello base station
    {{0xC0, 0xD5, 0xD5, 0xD5, 0xD5}} // blue base station
}};

/**
 * @brief The radio address of each robot
 * 
 */
static constexpr std::array<std::array<std::array<uint8_t, 5>, kNumShells>, 2> kRobotAddresses = {{
    {{
        {{0xC0, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-0
        {{0xC1, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-1
        {{0xC2, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-2
        {{0xC3, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-3
        {{0xC4, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-4
        {{0xC5, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-5
        {{0xC6, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-6
        {{0xC7, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-7
        {{0xC8, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-8
        {{0xC9, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-9
        {{0xCA, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-10
        {{0xCB, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-11
        {{0xCC, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-12
        {{0xCD, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-13
        {{0xCE, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-14
        {{0xCF, 0xC3, 0xC3, 0xC3, 0xC3}}, // yellow-15
    }},
    {{
        {{0xD0, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-0
        {{0xD1, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-1
        {{0xD2, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-2
        {{0xD3, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-3
        {{0xD4, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-4
        {{0xD5, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-5
        {{0xD6, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-6
        {{0xD7, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-7
        {{0xD8, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-8
        {{0xD9, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-9
        {{0xDA, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-10
        {{0xDB, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-11
        {{0xDC, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-12
        {{0xDD, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-13
        {{0xDE, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-14
        {{0xDF, 0xD5, 0xD5, 0xD5, 0xD5}}, // blue-15
    }}
}};

/**
* Special enum to dictate the special mode the robot should be in.
*
* In general, software should never be using anything except Default unless
* firmware creates special states for software, however, I included this in
* the C++ section to make sure the commands in software are still parallel
* to the commands in firmware.
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

struct ControlMessage {
    // The size of a control message
    static constexpr size_t kSize = 10;
    // The scale factor to and from message velocities
    static constexpr double kVelocityScaleFactor = 1000.0;

    /**
     * @brief Convert a ros message into a rtp Control Message
     * 
     * @param robot_id The robot id for the message
     * @param blue_team Should the message be for the blue team
     * @param motion_setpoint The motion setpoint for the message
     * @param manipulator_setpoint The manipulator setpoint for the message
     * @return ControlMessage 
     */
    static ControlMessage from_ros(
        unsigned int robot_id,
        bool blue_team,
        const std::shared_ptr<rj_msgs::msg::MotionSetpoint>& motion_setpoint,
        const std::shared_ptr<rj_msgs::msg::ManipulatorSetpoint>& manipulator_setpoint
    ) {

        return {
            manipulator_setpoint->trigger_mode,
            manipulator_setpoint->shoot_mode,
            robot_id,
            static_cast<unsigned int>(blue_team),
            static_cast<int16_t>(motion_setpoint->velocity_x_mps * kVelocityScaleFactor),
            static_cast<int16_t>(motion_setpoint->velocity_y_mps * kVelocityScaleFactor),
            static_cast<int16_t>(motion_setpoint->velocity_z_radps * kVelocityScaleFactor),
            static_cast<int8_t>(manipulator_setpoint->dribbler_speed * 100), // Percent from -100%->100%
            static_cast<uint8_t>(manipulator_setpoint->kick_strength * 100), // Percent from 0-100%
            0,
            0
        };
    }

    // 0 -> StandDown, 1 -> Immediate, 2 -> OnBreakBeam
    unsigned trigger_mode: 2;
    // 0 -> Kick; 1 -> Chip
    unsigned shoot_mode: 1;
    // Id of the robot
    unsigned robot_id: 4;
    // Team of the robot (0: Blue) (1: Yellow)
    unsigned team: 1;
    // x velocity (body frame)
    int16_t body_x;
    // y velocity (body frame)
    int16_t body_y;
    // w velocity (body frame)
    int16_t body_w;
    // speed of the dribbler
    int8_t dribbler_speed;
    // strenght of the kicker
    uint8_t kick_strength;
    // Unused bytes
    unsigned unused : 6;
    // Robot role
    unsigned role : 2;
} __attribute__((packed));

struct RobotStatusMessage {
    // The size of the RobotStatusMessage in bytes
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
        status.kicker_charged = kicker_charged;
        status.has_ball_sense = has_ball_sense;
        status.blue_team = team;
        status.battery_percent = battery_percent;
        std::array<bool, 5> m_errors = {false};
        for (size_t i = 0; i < 5; i++) {
            m_errors[i] = (motor_errors & (1 << i)) > 0;
        }
        status.motor_errors = m_errors;
        return status;
    }

    // True if the kicker is healthy
    unsigned kicker_healthy: 1;
    // True if is kicking
    unsigned kicker_charged: 1;
    // True if the robot has ball sense
    unsigned has_ball_sense: 1;
    // Id of the robot
    unsigned robot_id: 4;
    // Team of the robot (0: Blue) (1: Yellow)
    unsigned team: 1;
    // Battery Voltage
    uint8_t battery_percent;
    // Unused data
    unsigned unused: 3;
    // Error per motor
    unsigned motor_errors: 5;
} __attribute__((packed));

}  // namespace rtp