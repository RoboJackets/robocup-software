#pragma once

#include <rj_topic_utils/async_message_queue.h>

#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <vector>

#include "motion_control.hpp"
#include "node.hpp"

/**
 * Handles motion control for all robots. Calling this once will run motion
 * control on all robots.
 */
class MotionControlNode : public Node {
public:
    explicit MotionControlNode(Context* context);

    void run() override;

private:
    void run_motion(const WorldState& world_state, const GameState& game_state,
                   const std::array<Planning::Trajectory, kNumShells>& paths,
                   const std::array<bool, kNumShells>& joystick_controlled,
                   std::array<MotionSetpoint, kNumShells>* setpoints);
    Context* context_;
    std::vector<MotionControl> controllers_{};
    using WorldStateMsg = rj_msgs::msg::WorldState;
    using AsyncWorldStateMsgQueue = rj_topic_utils::AsyncMessageQueue<
        WorldStateMsg, rj_topic_utils::MessagePolicy::kLatest>;
    AsyncWorldStateMsgQueue::UniquePtr world_state_queue_;
};
