
#include "planning/MotionConstraints.hpp"

REGISTER_CONFIGURABLE(MotionConstraints);

ConfigDouble* MotionConstraints::_max_acceleration;
ConfigDouble* MotionConstraints::_max_speed;
ConfigDouble* MotionConstraints::_max_rotation_speed;
ConfigDouble* MotionConstraints::_max_rotation_acceleration;
ConfigDouble* MotionConstraints::_replan_threshold;
ConfigDouble* MotionConstraints::_replan_lead_time;

void MotionConstraints::createConfiguration(Configuration* cfg) {
    _max_acceleration =
        new ConfigDouble(cfg, "MotionConstraints/Max Acceleration", .5);
    _max_speed = new ConfigDouble(cfg, "MotionConstraints/Max Velocity", 2.0);
    _max_rotation_speed = new ConfigDouble(
        cfg, "MotionConstraints/Max Rotation Speed", 2.0);  //  radians / second
    _max_rotation_acceleration = new ConfigDouble(
            cfg, "MotionConstraints/Max Rotation Acceleration", 2);  //  radians / second^2
    _replan_threshold = new ConfigDouble(
        cfg, "MotionConstraints/Replan Threshold", 0.1);  //  meters
    _replan_lead_time =
        new ConfigDouble(cfg, "MotionConstraints/Replan Lead Time", 0);
}

MotionConstraints::MotionConstraints() {
    maxSpeed = *_max_speed;
    maxAcceleration = *_max_acceleration;
}
