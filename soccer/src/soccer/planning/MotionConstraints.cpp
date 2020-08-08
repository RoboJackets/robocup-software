
#include "planning/MotionConstraints.hpp"

REGISTER_CONFIGURABLE(MotionConstraints);

ConfigDouble* MotionConstraints::max_acceleration_config;
ConfigDouble* MotionConstraints::max_speed_config;

void MotionConstraints::create_configuration(Configuration* cfg) {
    max_acceleration_config = new ConfigDouble(cfg, "MotionConstraints/Max Acceleration", .5);
    max_speed_config = new ConfigDouble(cfg, "MotionConstraints/Max Velocity", 2.0);
}

MotionConstraints::MotionConstraints() {
    max_speed = *max_speed_config;
    max_acceleration = *max_acceleration_config;
}
