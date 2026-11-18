#pragma once

#include <cstdint>

namespace RadioMessage {

struct RobotStatusMessage {
    static constexpr float BATTERY_SCALE_FACTOR = 0.09884f;

    // True if the kicker is healthy
    unsigned kick_healthy : 1;
    // True if is kicking
    unsigned kick_status : 1;
    // True if the robot has ball sense
    unsigned ball_sense_status : 1;
    // Id of the robot
    unsigned robot_id : 4;
    // Team of the robot (0: Blue) (1: Yellow)
    unsigned team : 1;
    // Battery Voltage
    uint8_t battery_voltage;
    // Unused data
    unsigned unused : 2;
    // FPGA is working
    unsigned fpga_status : 1;
    // Error per motor
    unsigned motor_errors : 5;
} __attribute__((packed));

}  // namespace RadioMessage