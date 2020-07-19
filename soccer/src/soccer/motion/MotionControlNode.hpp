#pragma once

#include <rj_topic_utils/async_message_queue.h>

#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <vector>

#include "MotionControl.hpp"
#include "Node.hpp"

/**
 * Handles motion control for all robots. Calling this once will run motion
 * control on all robots.
 */
class MotionControlNode : public Node {
public:
    explicit MotionControlNode(Context* context);

    void run() override;

private:
    void runMotion(const WorldState& world_state, const GameState& game_state,
                   const std::array<Planning::Trajectory, Num_Shells>& paths,
                   const std::array<bool, Num_Shells>& joystick_controlled,
                   std::array<MotionSetpoint, Num_Shells>* setpoints);
    Context* _context;
    std::vector<MotionControl> _controllers{};
    using WorldStateMsg = rj_msgs::msg::WorldState;
    using AsyncWorldStateMsgQueue = rj_topic_utils::AsyncMessageQueue<
        WorldStateMsg, rj_topic_utils::MessagePolicy::kLatest>;
    AsyncWorldStateMsgQueue::UniquePtr world_state_queue_;
};
