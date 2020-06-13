#include "MotionControlNode.hpp"
#include "Robot.hpp"

MotionControlNode::MotionControlNode(Context* context) : _context(context) {
    _controllers.reserve(Num_Shells);
    for (int i = 0; i < Num_Shells; i++) {
        _controllers.emplace_back(MotionControl(context, i));
    }
}

void MotionControlNode::run() {
    runMotion(_context->world_state, _context->game_state, _context->trajectories,
              _context->is_joystick_controlled, &_context->motion_setpoints);
}

void MotionControlNode::runMotion(
    const WorldState& world_state, const GameState& game_state,
    const std::array<Planning::Trajectory, Num_Shells>& trajectories,
    const std::array<bool, Num_Shells>& joystick_controlled,
    std::array<MotionSetpoint, Num_Shells>* setpoints) {
    bool force_stop = game_state.state == GameState::State::Halt;
    for (int i = 0; i < Num_Shells; i++) {
        MotionControl& controller = _controllers[i];
        MotionSetpoint* setpoint = &(*setpoints)[i];
        if (force_stop) {
            controller.stop(setpoint);
        } else {
            controller.run(world_state.get_robot(true, i), trajectories[i],
                           joystick_controlled[i], setpoint);
        }
    }
}
