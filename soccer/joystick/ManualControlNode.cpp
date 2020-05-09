#include "ManualControlNode.hpp"
#include "GamepadController.hpp"
#include "InputDeviceManager.hpp"


ManualControlNode::ManualControlNode(Context* context) : _context(context) {
    // initialise device manager
    InputDeviceManager::setupInputDevices(_context->is_joystick_controlled);
}



ManualControlNode::run(Context* context) : _context(context) {
    InputDeviceManager::(_context);
}

