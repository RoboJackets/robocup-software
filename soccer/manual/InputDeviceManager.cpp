#include <manual/InputDeviceManager.hpp>
#include <manual/GamepadController.hpp>
#include <manual/GamepadJoystick.hpp>
#include <manual/SpaceNavJoystick.hpp>

InputDeviceManager::InputDeviceManager() {
  setupInputDevices();
}

void InputDeviceManager::setupInputDevices() {
  _inputDevices.clear();

  // initialize using the SDL joystick
  if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
    cerr << "ERROR: SDL could not initialize game controller system! SDL "
      "Error: " << SDL_GetError() << endl;
    return;
  }

  // Attempt to add additional mappings (relative to run)
  if (SDL_GameControllerAddMappingsFromFile(
                                            ApplicationRunDirectory()
                                            .filePath("../external/sdlcontrollerdb/gamecontrollerdb.txt")
                                            .toStdString()
                                            .c_str()) == -1) {
    cout << "Failed adding additional SDL Gamecontroller Mappings: "
         << SDL_GetError() << endl;
  }

  _manualID = -1;
  _multipleManual = false;
  _dampedTranslation = true;
  _dampedRotation = true;


  // TODO robots per team seems hard coded
  for (int i = 0; i < Robots_Per_Team; i++) {
    _inputDevices.push_back(new GamepadController());
  }

  //_joysticks.push_back(new SpaceNavJoystick()); //Add this back when
  // isValid() is working properly
}

void InputDeviceManager::update() {
  // TODO This is where the bulk of event handling and checking should be done for new controllers
  // Check for sdl device connected

  // Pump sdl update
  // Iterate eventqueue
  // For each event
  // If it is a controller connected event make a new gamepad object and register it
  // if it is a Joystick disconnected delete the pointer to the joystick and let the destructor handle the rest
  // if it is a Joystick event of
  // if (SDL_HasEvents(SDL_CONTROLLERAXISMOTION, SDL_CONTROLLERBUTTONUP))
  // send the event to the to the controllers update function



  for (InputDevice* inputDevice : _inputDevices) {
    inputDevice->update();
  }
}

bool InputDeviceManager::joystickValid() const {
    for (InputDevice* dev : _inputDevices) {
        if (dev->valid()) return true;
    }
    return false;
}

void InputDeviceManager::manualID(int value) {
  QMutexLocker locker(&_loopMutex);
  _manualID = value;

  for (InputDevice* dev : _inputDevices) {
    dev->reset();
  }
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
      vals.push_back(getInputDeviceControlValue(*joy));
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
void InputDeviceManager::applyInputDeviceControls(OurRobot* r, Packet::Control* tx) {

    std::vector<int> manualIds = _inputDeviceManager->getInputDeviceRobotIds();
    // MANUAL STUFF
    if (_multipleManual) {
        auto info =
            find(manualIds.begin(), manualIds.end(), r->shell());
        int index = info - manualIds.begin();

        // figure out if this shell value has been assigned to a
        // joystick
        // do stuff with that information such as assign it to the first
        // available
        if (info == manualIds.end()) {
            for (int i = 0; i < manualIds.size(); i++) {
                if (manualIds[i] == -1) {
                    index = i;
                    _inputDevices[i]->setRobotId(r->shell());
                    manualIds[i] = r->shell();
                    break;
                }
            }
        }

        if (index < manualIds.size()) {
            applyInputDeviceControls(
                getInputDeviceControlValue(*_inputDevices[index]),
                txRobot->mutable_control(), r);
        }
    } else if (_manualID == r->shell()) {
        auto controlValues = getInputDeviceControlValues();
        if (controlValues.size()) {
            applyInputDeviceControls(controlValues[0],
                                  txRobot->mutable_control(), r);
        }
    }

    Geometry2d::Point translation(controlVals.translation);

    // use world coordinates if we can see the robot
    // otherwise default to body coordinates
    if (robot && robot->visible && _useFieldOrientedManualDrive) {
        translation.rotate(-M_PI / 2 - robot->angle);
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
