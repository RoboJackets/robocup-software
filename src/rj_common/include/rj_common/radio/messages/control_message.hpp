#pragma once

#include <cstdint>

namespace RadioMessage {

struct ControlMessage {
    static constexpr float VELOCITY_SCALE_FACTOR = 1000.0f;

    // TODO: Determine the mapping for this
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

} // namespace messagea