#include "ManualControlNode.hpp"


ManualControlNode::ManualControlNode(Context* context) : _context(context) {
    // initialise device manager
    manager = new InputDeviceManager(_context);
    manager->setupInputDevices(_context->is_joystick_controlled);
}


void ManualControlNode::run() {
    manager->update(_context->robot_intents, _context->is_joystick_controlled);
}

