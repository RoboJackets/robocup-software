#pragma once

#include <configuration.hpp>

struct RotationConstraints {
public:
    RotationConstraints()
        : max_speed(*max_rotation_speed),
          max_accel(*max_rotation_acceleration) {}
    double max_speed;
    double max_accel;

    static void create_configuration(Configuration* cfg);
    static ConfigDouble* max_rotation_speed;
    static ConfigDouble* max_rotation_acceleration;
};