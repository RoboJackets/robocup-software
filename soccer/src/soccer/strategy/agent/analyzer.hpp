#pragma once

#include <cstdlib>
#include <vector>

#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/alive_robots.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

#include "strategy/agent/position/position.hpp"
#include "strategy/agent/position/goalie.hpp"
#include "strategy/agent/position/offense.hpp"
#include "strategy/agent/position/defense.hpp"

#include "robot_intent.hpp"
#include "world_state.hpp"

// tell compiler this class exists, but no need to import the whole header
class AgentActionClient;

namespace strategy {

/*
 * Analyzer is an abstract superclass. Its subclasses handle strategy logic.
 * The goal is to isolate the strategy logic from the ROS interfacing. This
 * allows us to swap Analyzer classes at runtime. (Google "Strategy Design
 * Pattern" for more info.)
 *
 * A good analogy is how the Planner Node uses the various Planner objects
 * (PathTargetPlanner, etc.). The Planner objects take in a plan request and
 * output a Trajectory. Planner objects don't know anything about ROS; that is
 * all handled by the Planner Node.
 */
class Analyzer {
public:
    Analyzer(int r_id);

    /**
     * @brief return a Position based on the current world state?
     */

    std::unique_ptr<Position> get_init_position(WorldState& world_state, FieldDimensions& field_dimensions);

    std::optional<RobotIntent> get_task(WorldState& world_state, FieldDimensions& field_dimensions);

private:
    std::unique_ptr<Position> current_position_;

    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;

    /*
     * @brief assert world_state is valid before using it in get_task().
     *
     * Since ROS nodes launch in a random order, often the agents will launch
     * before they have any world_state info to act on. Thus, we must return
     * NOPs to the robots until vision_filter node starts up.
     *
     * @return false if world_state is invalid (nullptr), true otherwise
     */
    bool assert_world_state_valid();

    // const because should never be changed, but initializer list will allow
    // us to set this once initially
    const int robot_id_;

    // protected to allow WorldState to be accessed directly by derived
    WorldState* last_world_state_;
};

}  // namespace strategy
