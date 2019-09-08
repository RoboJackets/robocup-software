
#include "manual/InputDevice.hpp"

REGISTER_CONFIGURABLE(InputDevice);

ConfigDouble* InputDevice::InputDeviceRotationMaxSpeed;
ConfigDouble* InputDevice::InputDeviceRotationMaxDampedSpeed;
ConfigDouble* InputDevice::InputDeviceTranslationMaxSpeed;
ConfigDouble* InputDevice::InputDeviceTranslationMaxDampedSpeed;

void InputDevice::createConfiguration(Configuration* cfg) {
    InputDeviceRotationMaxSpeed =
        new ConfigDouble(cfg, "InputDevice/Max Rotation Speed", .5);
    InputDeviceRotationMaxDampedSpeed =
        new ConfigDouble(cfg, "InputDevice/Max Damped Rotation Speed", .25);
    InputDeviceTranslationMaxSpeed =
        new ConfigDouble(cfg, "InputDevice/Max Translation Speed", 3.0);
    InputDeviceTranslationMaxDampedSpeed =
        new ConfigDouble(cfg, "InputDevice/Max Damped Translation Speed", 1.0);
}
