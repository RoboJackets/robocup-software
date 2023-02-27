#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"

namespace strategy {

/**
 * RoleInterface is an abstract superclass. Its subclasses handle strategy logic.
 * The goal is to isolate the strategy logic from ROS interfacing. This allows us
 * swap Role classes at runtime. (Google "Strategy Design Pattern" for more info)
 * 
 * A good analogy is how the Planner Node uses the various Planner objects
 * (PathTargetPlanner, etc.). The Planner objects take in a plan request and
 * output a Trajectory. Planner objects don't know anything about ROS; that is
 * all handled by the Planner Node.
 * 
 * Similiarly, this abstract superclass serves as an interface for all roles 
 * (ie. Waller, Marker, Seeker, etc.) to follow when being created. These roles 
 * will take in a robot intent from the Defense/Offense classes and output a new 
 * robot intent with the robot's next point depending on their role. The Defense/
 * Offense classes will decide on what role the robot is currently playing based 
 * on the game situation (such as location, etc.) This design pattern allows for 
 * more flexibility in our usage.
 */
class RoleInterface {
public:
    /**
     * @brief Returns a behavior based on the role
     *
     * @param [RobotIntent intent] [RobotIntent of the Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    virtual std::optional<RobotIntent> get_task(RobotIntent intent,
                                                const WorldState* const world_state) = 0;


private:
};

}  // namespace strategy
