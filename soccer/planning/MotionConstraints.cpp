
#include "planning/MotionConstraints.hpp"


REGISTER_CONFIGURABLE(MotionConstraints);

ConfigDouble *MotionConstraints::_max_acceleration;
ConfigDouble *MotionConstraints::_max_speed;
ConfigDouble *MotionConstraints::_max_angle_speed;
ConfigDouble *MotionConstraints::_replan_threshold;

void MotionConstraints::createConfiguration(Configuration *cfg) {
    _max_acceleration   = new ConfigDouble(cfg, "MotionConstraints/Max Acceleration", .5);
    _max_speed          = new ConfigDouble(cfg, "MotionConstraints/Max Velocity", 2.0);
    _max_angle_speed      = new ConfigDouble(cfg, "MotionConstraints/Max Angle Speed", 80); //  radians / second
    _replan_threshold      = new ConfigDouble(cfg, "MotionConstraints/Replan Threshold", 0.1); //  meters
}

MotionConstraints::MotionConstraints() {
    maxSpeed = *_max_speed;
    maxAcceleration = *_max_acceleration;
    maxAngleSpeed = *_max_angle_speed;
}
