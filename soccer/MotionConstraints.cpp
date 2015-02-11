
#include "MotionConstraints.hpp"


REGISTER_CONFIGURABLE(MotionConstraints);

ConfigDouble *MotionConstraints::_max_acceleration;
ConfigDouble *MotionConstraints::_max_speed;
ConfigDouble *MotionConstraints::_max_angle_speed;
ConfigDouble *MotionConstraints::_max_replan_distance;

void MotionConstraints::createConfiguration(Configuration *cfg) {
    _max_acceleration   = new ConfigDouble(cfg, "MotionConstraints/Max Acceleration", .5);
    _max_speed          = new ConfigDouble(cfg, "MotionConstraints/Max Velocity", 2.0);
    _max_angle_speed      = new ConfigDouble(cfg, "MotionConstraints/Max Angle Speed", 80); //  degrees / second
    _max_replan_distance      = new ConfigDouble(cfg, "MotionConstraints/Max Replan Distance", 0.6); //  degrees / second
}

MotionConstraints::MotionConstraints() {
    maxSpeed = *_max_speed;
    maxAcceleration = *_max_acceleration;
    maxAngleSpeed = *_max_angle_speed;
    maxReplanDistance = *_max_replan_distance;
}
