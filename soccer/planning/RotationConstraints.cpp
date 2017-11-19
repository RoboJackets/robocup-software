
#include "planning/RotationConstraints.hpp"

REGISTER_CONFIGURABLE(RotationConstraints);

ConfigDouble* RotationConstraints::_max_rotation_speed;
ConfigDouble* RotationConstraints::_max_rotation_acceleration;

void RotationConstraints::createConfiguration(Configuration* cfg) {
    _max_rotation_speed =
        new ConfigDouble(cfg, "RotationConstraints/Max Rotation Speed",
                         2.0);  //  radians / second
    _max_rotation_acceleration =
        new ConfigDouble(cfg, "RotationConstraints/Max Rotation Acceleration",
                         2);  //  radians / second^2
}