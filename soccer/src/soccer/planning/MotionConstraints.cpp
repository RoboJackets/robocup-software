
#include "planning/MotionConstraints.hpp"

REGISTER_CONFIGURABLE(MotionConstraints);

ConfigDouble* MotionConstraints::max_acceleration;
ConfigDouble* MotionConstraints::max_speed;

void MotionConstraints::createConfiguration(Configuration* cfg) {
    max_acceleration =
        new ConfigDouble(cfg, "MotionConstraints/Max Acceleration", .5);
    max_speed = new ConfigDouble(cfg, "MotionConstraints/Max Velocity", 2.0);
}

MotionConstraints::MotionConstraints() {
    maxSpeed = *max_speed;
    maxAcceleration = *max_acceleration;
}
