#include "InputDeviceManager.hpp"
#include "GamepadController.hpp"
#include <SDL.h>

InputDeviceManager::InputDeviceManager(Context* context) : _context(context) {
  setupInputDevices(_context->is_joystick_controlled);
}

void InputDeviceManager::setupInputDevices(
    std::array<bool, Num_Shells>& _is_joystick_controlled) {
    _inputDevices.clear();
    _inputDevices.reserve(Num_Shells);

    // Init event system
    if (SDL_Init(SDL_INIT_EVENTS) != 0) {
        cerr << "ERROR: SDL could not Initialize event system! SDL "
            "Error: " << SDL_GetError() << endl;
        return;
    }

    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        cerr << "ERROR: SDL could not initialize game controller! SDL "
            "Error: " << SDL_GetError() << endl;
    }
    
    _manualID = -1;
    _multipleManual = false;
    _dampedTranslation = true;
    _dampedRotation = true;
}

void InputDeviceManager::update(
    std::array<RobotIntent, Num_Shells>& _robot_intents,
    std::array<bool, Num_Shells>& _is_joystick_controlled) {
    // Pump sdl update for each type
    SDL_GameControllerUpdate();

    SDL_Event event;

    // Iterate eventqueue
    while (SDL_PollEvent(&event)) {
        // Figure out what event it is
        switch (event.type) {

        // If it is a controller connected event make a new gamepad object and register it
        case SDL_CONTROLLERDEVICEADDED:
            {
                int sdl_index = event.cdevice.which;
                if (sdl_index < SDL_NumJoysticks()) {
                    _inputDevices.insert(_inputDevices.begin() + sdl_index, new GamepadController(event));
                } else {
                    cerr << "ERROR: Attempted to open a controller beyond the number of controllers"
                        << SDL_GetError() << endl;
                }
            }
            break;

        // if it is a Joystick disconnected delete the pointer to the joystick and let the destructor handle the rest
        case SDL_CONTROLLERDEVICEREMOVED:
            {
                if (event.cdevice.which < SDL_NumJoysticks()) {
                    delete _inputDevices.at(event.cdevice.which);
                } else {
                    cerr << "ERROR: Attempted to close a controller beyond the number of controllers"
                        << SDL_GetError() << endl;
                }
            }
            break;

        // if it is a Joystick event of
        case SDL_CONTROLLERAXISMOTION:
        case SDL_CONTROLLERBUTTONUP:
            // send the event to the to the controllers update function
            _inputDevices.at(event.cdevice.which)->update(event);
            break;
        }
    }

    // Apply input device updates to robots
    applyInputDeviceControls(_robot_intents, _is_joystick_controlled);

}

bool InputDeviceManager::joystickValid() const {
    for (InputDevice* dev : _inputDevices) {
        if (dev->valid()) return true;
    }
    return false;
}

void InputDeviceManager::manualID(int value) {
  // QMutexLocker locker(&_loopMutex);
  _manualID = value;

  for (InputDevice* dev : _inputDevices) {
    dev->reset();
  }
}

void InputDeviceManager::dampedRotation(bool value) {
  // QMutexLocker locker(&_loopMutex);
  _dampedRotation = value;
}

void InputDeviceManager::dampedTranslation(bool value) {
  // QMutexLocker locker(&_loopMutex);
  _dampedTranslation = value;
}


vector<int> InputDeviceManager::getInputDeviceRobotIds() {
  vector<int> robotIds;
  for (InputDevice* dev : _inputDevices) {
    if (dev->valid()) {
      robotIds.push_back(dev->getRobotId());
    } else {
      robotIds.push_back(-2);
    }
  }
  return robotIds;
}

std::vector<InputDeviceControlValues> InputDeviceManager::getInputDeviceControlValues() {
  std::vector<InputDeviceControlValues> vals;
  for (InputDevice* dev : _inputDevices) {
    if (dev->valid()) {
      vals.push_back(getInputDeviceControlValue(*dev));
    }
  }
  return vals;
}


InputDeviceControlValues InputDeviceManager::getInputDeviceControlValue(InputDevice& dev) {
    InputDeviceControlValues vals = dev.getInputDeviceControlValues();
    if (dev.valid()) {
        // keep it in range
        vals.translation.clamp(sqrt(2.0));
        if (vals.rotation > 1) vals.rotation = 1;
        if (vals.rotation < -1) vals.rotation = -1;

        // Gets values from the configured joystick control
        // values,respecting damped
        // state
        if (_dampedTranslation) {
            vals.translation *=
                InputDevice::InputDeviceTranslationMaxDampedSpeed->value();
        } else {
            vals.translation *= InputDevice::InputDeviceTranslationMaxSpeed->value();
        }
        if (_dampedRotation) {
            vals.rotation *= InputDevice::InputDeviceRotationMaxDampedSpeed->value();
        } else {
            vals.rotation *= InputDevice::InputDeviceRotationMaxSpeed->value();
        }

        // scale up kicker and dribbler speeds
        vals.dribblerPower *= Max_Dribble;
        vals.kickPower *= Max_Kick;
    }
    return vals;
}


void InputDeviceManager::applyInputDeviceControls(
    std::array<RobotIntent, Num_Shells>& robot_intents,
    const std::array<bool, Num_Shells>& _is_joystick_controlled) {
    for (InputDevice* dev: _inputDevices) {
        int targetId = dev->getRobotId();
        // check if dev is valid and robot is supposed to be manually controlled
        if (dev->valid() && _is_joystick_controlled[targetId]) {
            InputDeviceControlValues val = getInputDeviceControlValue(*dev);
            RobotIntent& intent = robot_intents[targetId];
            Planning::WorldVelTargetCommand* cmd = new Planning::WorldVelTargetCommand(val.translation);
            intent.motion_command = cmd->clone();
            // intent.rotation_command = new FacePointCommand(val.rotation);
            if (val.kick) {
                intent.shoot_mode = RobotIntent::ShootMode::KICK;
            } else if (val.chip) {
                intent.shoot_mode = RobotIntent::ShootMode::CHIP;
            }
            intent.kcstrength = val.kickPower;
        }
    }

}

