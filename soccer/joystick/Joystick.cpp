
#include "joystick/Joystick.hpp"

REGISTER_CONFIGURABLE(Joystick);

ConfigDouble* Joystick::JoystickRotationMaxSpeed;
ConfigDouble* Joystick::JoystickRotationMaxDampedSpeed;
ConfigDouble* Joystick::JoystickTranslationMaxSpeed;
ConfigDouble* Joystick::JoystickTranslationMaxDampedSpeed;

void Joystick::createConfiguration(Configuration* cfg) {
    JoystickRotationMaxSpeed =
        new ConfigDouble(cfg, "Joystick/Max Rotation Speed", .5);
    JoystickRotationMaxDampedSpeed =
        new ConfigDouble(cfg, "Joystick/Max Damped Rotation Speed", .25);
    JoystickTranslationMaxSpeed =
        new ConfigDouble(cfg, "Joystick/Max Translation Speed", 3.0);
    JoystickTranslationMaxDampedSpeed =
        new ConfigDouble(cfg, "Joystick/Max Damped Translation Speed", 1.0);
}
