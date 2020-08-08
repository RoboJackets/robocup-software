#pragma once

#include <rj_topic_utils/async_message_queue.h>

#include <context.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <vector>

#include "node.hpp"
#include "trajectory.hpp"
#include "planner/plan_request.hpp"
#include "planner/planner.hpp"

namespace Planning {

class PlannerForRobot {
public:
    PlannerForRobot();

    Trajectory plan_for_robot(const Planning::PlanRequest& request);

private:
    std::vector<std::unique_ptr<Planner>> planners_;
};

class PlannerNode : public Node {
public:
    PlannerNode(Context* context);

    void run() override;

private:
    Context* context_;

    std::vector<PlannerForRobot> robots_planners_;

    using WorldStateMsg = rj_msgs::msg::WorldState;
    using AsyncWorldStateMsgQueue = rj_topic_utils::AsyncMessageQueue<
        WorldStateMsg, rj_topic_utils::MessagePolicy::kLatest>;
    AsyncWorldStateMsgQueue::UniquePtr world_state_queue_;
};

}  // namespace Planning
