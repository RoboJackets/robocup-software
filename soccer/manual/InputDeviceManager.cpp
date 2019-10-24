#include <manual/InputDeviceManager.hpp>
#include <manual/GamepadController.hpp>
#include <manual/GamepadJoystick.hpp>
#include <manual/SpaceNavJoystick.hpp>

InputDeviceManager::InputDeviceManager() {
  setupInputDevices();
}

void InputDeviceManager::setupInputDevices() {
    _inputDevices.clear();

    // Init event system
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        cerr << "ERROR: SDL could not Initialize event system! SDL "
            "Error: " << SDL_GetError() << endl;
        return;
    }

    //TODO
    // run static init functions for each type of devices
    GamepadController::initDeviceType();
    GamepadJoystick::initDeviceType();

    _manualID = -1;
    _multipleManual = false;
    _dampedTranslation = true;
    _dampedRotation = true;

    // _inputDevices.resize(Robots_Per_Team);
}

void InputDeviceManager::update(std::vector<OurRobot*>& robots, Packet::RadioTx* tx) {
    // TODO This is where the bulk of event handling and checking should be done for new controllers
    // Pump sdl update for each type
    SDL_GameControllerUpdate();

    // TODO Only serve so many events
    SDL_Event event;

    // TODO Opens on number of robots but currently indexes by SDL connection number
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
    applyInputDeviceControls(robots, tx);

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


// TODO Remove joystick controlVals from pass and use in header
void InputDeviceManager::applyInputDeviceControls(std::vector<OurRobot*>& robots, Packet::RadioTx* tx) {

    // Add RadioTx commands for visible robots and apply joystick input
    // TODO I think this radio tx application needs to be removed and only the final packets passed here
    // TODO I think this needs to be moved outside this
    Packet::Robot* txRobot = tx->add_robots();
    for (OurRobot* r : robots) {
        if (r->visible() || _manualID == r->shell()) {

            // Copy motor commands.
            // Even if we are using the joystick, this sets robot_id and the
            // number of motors.
            txRobot->CopyFrom(r->robotPacket);
        }
    }
    for (InputDevice* dev : _inputDevices) {
        int robot_id = dev->getRobotId();
        if (robot_id != -1) {
            applyInputDeviceControls(dev->getInputDeviceControlValues(),
                                     txRobot->mutable_control(), robots.at(robot_id));
        }
    }
}

void InputDeviceManager::applyInputDeviceControls(const InputDeviceControlValues& controlVals, Packet::Control* tx, OurRobot* robot) {

    Geometry2d::Point translation(controlVals.translation);

    // use world coordinates if we can see the robot
    // otherwise default to body coordinates
    if (robot && robot->visible() && _useFieldOrientedManualDrive) {
      translation.rotate(-M_PI / 2 - robot->angle());
    }

    // translation
    tx->set_xvelocity(translation.x());
    tx->set_yvelocity(translation.y());

    // rotation
    tx->set_avelocity(controlVals.rotation);

    // kick/chip
    bool kick = controlVals.kick || controlVals.chip;
    tx->set_triggermode(kick
                            ? (_kickOnBreakBeam ? Packet::Control::ON_BREAK_BEAM
                                                : Packet::Control::IMMEDIATE)
                            : Packet::Control::STAND_DOWN);
    tx->set_kcstrength(controlVals.kickPower);
    tx->set_shootmode(controlVals.kick ? Packet::Control::KICK
                                       : Packet::Control::CHIP);

    // dribbler
    tx->set_dvelocity(controlVals.dribble ? controlVals.dribblerPower : 0);
}
