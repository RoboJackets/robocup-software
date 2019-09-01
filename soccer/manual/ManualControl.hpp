#pragma once

#include <vector>
#include <optional>
#include <string.h>

#include <Robot.hpp>
#include <RobotConfig.hpp>

using namespace std;

class Joystick;
struct JoystickControlValues;

class ManualManager {
protected:
  void applyJoystickControls(const JoystickControlValues& controlVals,
                            Packet::Control* txRobot, OurRobot* robot);

public:
  // joystick control
  std::vector<Joystick*> _joysticks;

  // Board ID of the robot to manually control or -1 if none
  int _manualID;
  // Use multiple joysticks at once
  bool _multipleManual;

  void setupJoysticks();
  bool joystickValid() const;

  void joystickKickOnBreakBeam(bool value);

  JoystickControlValues getJoystickControlValue(Joystick& joy);
  std::vector<JoystickControlValues> getJoystickControlValues();

  void manualID(int value);
  int manualID() const { return _manualID; }

  void multipleManual(bool value);
  bool multipleManual() const { return _multipleManual; }

  bool useFieldOrientedManualDrive() const {
    return _useFieldOrientedManualDrive;
  }
  void setUseFieldOrientedManualDrive(bool foc) {
    _useFieldOrientedManualDrive = foc;
  }

private:
  // joystick damping
  bool _dampedRotation;
  bool _dampedTranslation;

  bool _kickOnBreakBeam;

  // If true, rotates robot commands from the joystick based on its
  // orientation on the field
  bool _useFieldOrientedManualDrive = false;


  std::vector<int> getJoystickRobotIds();
}
