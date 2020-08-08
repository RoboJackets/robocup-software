#include "MotionControlNode.hpp"

#include <rj_constants/topic_names.hpp>

#include "Robot.hpp"

MotionControlNode::MotionControlNode(Context* context) : context_(context) {
    controllers_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        controllers_.emplace_back(MotionControl(context, i));
    }

    world_state_queue_ = std::make_unique<AsyncWorldStateMsgQueue>(
        "motion_control_game_state_sub", vision_filter::topics::kWorldStatePub);
}

void MotionControlNode::run() {
    const WorldStateMsg::SharedPtr world_state_msg = world_state_queue_->get();
    if (world_state_msg == nullptr) {
        return;
    }

    run_motion(rj_convert::convert_from_ros(*world_state_msg), context_->game_state,
               context_->trajectories, context_->is_joystick_controlled,
               &context_->motion_setpoints);
}

void MotionControlNode::run_motion(const WorldState& world_state, const GameState& game_state,
                                   const std::array<Planning::Trajectory, kNumShells>& trajectories,
                                   const std::array<bool, kNumShells>& joystick_controlled,
                                   std::array<MotionSetpoint, kNumShells>* setpoints) {
    bool force_stop = game_state.state == GameState::State::Halt;
    for (int i = 0; i < kNumShells; i++) {
        MotionControl& controller = controllers_[i];
        MotionSetpoint* setpoint = &(*setpoints)[i];
        if (force_stop) {
            controller.stop(setpoint);
        } else {
            controller.run(world_state.get_robot(true, i), trajectories[i], joystick_controlled[i],
                           setpoint);
        }
    }
}
