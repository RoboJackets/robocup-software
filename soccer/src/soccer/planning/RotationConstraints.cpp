
#include "planning/RotationConstraints.hpp"

REGISTER_CONFIGURABLE(RotationConstraints);

ConfigDouble* RotationConstraints::max_rotation_speed;
ConfigDouble* RotationConstraints::max_rotation_acceleration;

void RotationConstraints::create_configuration(Configuration* cfg) {
    max_rotation_speed = new ConfigDouble(cfg, "RotationConstraints/Max Rotation Speed",
                                          2.0);  //  radians / second
    max_rotation_acceleration =
        new ConfigDouble(cfg, "RotationConstraints/Max Rotation Acceleration",
                         2);  //  radians / second^2
}