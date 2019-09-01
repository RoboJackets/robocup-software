#include <joystick/GamepadController.hpp>
#include <joystick/GamepadJoystick.hpp>
#include <joystick/Joystick.hpp>
#include <joystick/SpaceNavJoystick.hpp>

void ManualControl::setupJoysticks() {
  _joysticks.clear();

  _manualID = -1;
  _multipleManual = false;
  _dampedTranslation = true;
  _dampedRotation = true;

  // TODO WTF global special use variables
  GamepadController::controllersInUse.clear();
  GamepadController::joystickRemoved = -1;

  // TODO robots per team seems hard coded
  for (int i = 0; i < Robots_Per_Team; i++) {
    _joysticks.push_back(new GamepadController());
  }

  //_joysticks.push_back(new SpaceNavJoystick()); //Add this back when
  // isValid() is working properly
}

void ManualControl::update() {
  for (Joystick* joystick : _joysticks) {
    joystick->update();
  }
  GamepadController::joystickRemoved = -1;
}

void Processor::manualID(int value) {
  QMutexLocker locker(&_loopMutex);
  _manualID = value;

  for (Joystick* joy : _joysticks) {
    joy->reset();
  }
}


void Processor::multipleManual(bool value) { _multipleManual = value; }


bool ManualControl::joystickValid() const {
  for (Joystick* joy : _joysticks) {
    if (joy->valid()) return true;
  }
  return false;
}


vector<int> ManualControl::getJoystickRobotIds() {
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

std::vector<JoystickControlValues> ManualControl::getJoystickControlValues() {
  std::vector<JoystickControlValues> vals;
  for (Joystick* joy : _joysticks) {
    if (joy->valid()) {
      vals.push_back(getJoystickControlValue(*joy));
    }
  }
  return vals;
}


JoystickControlValues ManualControl::getJoystickControlValue(Joystick& joy) {
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
void ManualControl::applyJoystickControls(const JoystickControlValues& controlVals,
                                      Packet::Control* tx, OurRobot* robot) {
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
