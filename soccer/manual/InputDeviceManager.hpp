#pragma once

#include <vector>
#include <optional>
#include <string.h>

#include <Robot.hpp>
#include <RobotConfig.hpp>

using namespace std;

class Input;
struct InputDeviceControlValues;

class InputDeviceManager {
protected:
  void applyInputDeviceControls(OurRobot* robot, Packet::Control* txRobot);

public:
  InputDeviceManager();

  // input devices index represents robot id
  std::vector<InputDevice*> _inputDevices;


  // Board ID of the robot to manually control or -1 if none
  int _manualID;
  // Use multiple joysticks at once
  bool _multipleManual;


  bool _kickOnBreakBeam = false;

  void setupInputDevices();

  bool joystickValid() const;

  void KickOnBreakBeam(bool value);

  InputDeviceControlValues getInputDeviceControlValue(InputDevice& joy);
  std::vector<InputDeviceControlValues> getInputDeviceControlValues();

  void dampedRotation(bool value);
  void dampedTranslation(bool value);

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


  std::vector<int> getInputDeviceRobotIds();
}
