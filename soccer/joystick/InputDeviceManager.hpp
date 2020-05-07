#pragma once

#include <vector>
#include <optional>
#include <string.h>

#include <Robot.hpp>
#include <RobotConfig.hpp>
#include "InputDevice.hpp"
#include "RobotIntent.hpp"
#include <Context.hpp>

using namespace std;

class Input;
struct InputDeviceControlValues;

class InputDeviceManager {
protected:
  void applyInputDeviceControls(OurRobot* robot, Packet::Control* txRobot);

public:
  InputDeviceManager(Context* context);

  // input devices index represents robot id
  std::vector<InputDevice*> _inputDevices;

  // Board ID of the robot to manually control or -1 if none
  int _manualID;
  // Use multiple joysticks at once
  bool _multipleManual;

  void setupInputDevices();

  void attachVirtual();

  void update(std::vector<OurRobot*>& robots, Packet::RadioTx* tx);

  bool joystickValid() const;

  void kickOnBreakBeam(bool value) { _kickOnBreakBeam = value; }

  InputDeviceControlValues getInputDeviceControlValue(InputDevice& dev);
  std::vector<InputDeviceControlValues> getInputDeviceControlValues();

  void dampedRotation(bool value);
  void dampedTranslation(bool value);

  void manualID(int value);
  int manualID() const { return _manualID; }

  bool multipleManual() const { return _multipleManual; }
  void multipleManual(bool value) { _multipleManual = value; }

  bool useFieldOrientedManualDrive() const {
    return _useFieldOrientedManualDrive;
  }
  void setUseFieldOrientedManualDrive(bool foc) {
    _useFieldOrientedManualDrive = foc;
  }

  void applyInputDeviceControls(std::array<RobotIntent, Num_Shells>& robot_intents,
    const std::array<bool, Num_Shells>& _is_joystick_controlled);

  std::vector<int> getInputDeviceRobotIds();

private:
  // joystick damping
  bool _dampedRotation;
  bool _dampedTranslation;

  bool _kickOnBreakBeam;

  // If true, rotates robot commands from the joystick based on its
  // orientation on the field
  bool _useFieldOrientedManualDrive = false;
  
  Context* _context;
};
