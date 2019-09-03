#include <input-devices/GamepadController.hpp>
#include <input-devices/GamepadJoystick.hpp>
#include <input-devices/InputDevice.hpp>
#include <input-devices/SpaceNavJoystick.hpp>

void ManualControl::setupInputDevices() {
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
  // for (int i = 0; i < Robots_Per_Team; i++) {
  //   _joysticks.push_back(new GamepadController());
  // }

  //_joysticks.push_back(new SpaceNavJoystick()); //Add this back when
  // isValid() is working properly
}

void InputDeviceManager::update() {
  // TODO This is where the bulk of event handling and checking should be done for new controllers
  // Check for sdl device connected

  if (connected) {
    // Check if dc
    if (joystickRemoved >= 0 && controllerId > joystickRemoved) {
      controllerId -= 1;
    }
    if (!SDL_GameControllerGetAttached(_controller)) {
      closeJoystick();
      return;
    }
  } else {
    // Check if new controller found
    // TODO use the SDL event API to only run this if we receive a connected
    // event.
    openJoystick();
    if (!connected) {
      return;
    }
  }


  for (InputDevice* inputDevice : _inputDevices) {
    inputDevice->update();
  }
  //TODO get rid of this
  GamepadController::joystickRemoved = -1;
}

void InputDeviceManager::manualID(int value) {
  QMutexLocker locker(&_loopMutex);
  _manualID = value;

  for (Joystick* joy : _joysticks) {
    joy->reset();
  }
}


void InputDeviceManager::multipleManual(bool value) { _multipleManual = value; }


vector<int> InputDeviceManager::getJoystickRobotIds() {
  vector<int> robotIds;
  for (Joystick* joy : _joysticks) {
    if (joy->valid()) {
      robotIds.push_back(joy->getRobotId());
    } else {
      robotIds.push_back(-2);
    }
  }
  return robotIds;
}

std::vector<JoystickControlValues> InputDeviceManager::getJoystickControlValues() {
  std::vector<JoystickControlValues> vals;
  for (Joystick* joy : _joysticks) {
    if (joy->valid()) {
      vals.push_back(getJoystickControlValue(*joy));
    }
  }
  return vals;
}


JoystickControlValues InputDeviceManager::getJoystickControlValue(Joystick& joy) {
    JoystickControlValues vals = joy.getJoystickControlValues();
    if (joy.valid()) {
        // keep it in range
        vals.translation.clamp(sqrt(2.0));
        if (vals.rotation > 1) vals.rotation = 1;
        if (vals.rotation < -1) vals.rotation = -1;

        // Gets values from the configured joystick control
        // values,respecting damped
        // state
        if (_dampedTranslation) {
            vals.translation *=
                Joystick::JoystickTranslationMaxDampedSpeed->value();
        } else {
            vals.translation *= Joystick::JoystickTranslationMaxSpeed->value();
        }
        if (_dampedRotation) {
            vals.rotation *= Joystick::JoystickRotationMaxDampedSpeed->value();
        } else {
            vals.rotation *= Joystick::JoystickRotationMaxSpeed->value();
        }

        // scale up kicker and dribbler speeds
        vals.dribblerPower *= Max_Dribble;
        vals.kickPower *= Max_Kick;
    }
    return vals;
}



// TODO Remove joystick controlVals from pass and use in header
void InputDeviceManager::applyJoystickControls(OurRobot* robot, Packet::Control* tx) {


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
                    _joysticks[i]->setRobotId(r->shell());
                    manualIds[i] = r->shell();
                    break;
                }
            }
        }

        if (index < manualIds.size()) {
            applyJoystickControls(
                getJoystickControlValue(*_joysticks[index]),
                txRobot->mutable_control(), r);
        }
    } else if (_manualID == r->shell()) {
        auto controlValues = getJoystickControlValues();
        if (controlValues.size()) {
            applyJoystickControls(controlValues[0],
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
