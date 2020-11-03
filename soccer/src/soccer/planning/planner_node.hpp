#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <context.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/goalie.hpp>

#include "node.hpp"
#include "planner/plan_request.hpp"
#include "planner/planner.hpp"
#include "robot_intent.hpp"
#include "trajectory.hpp"
#include "world_state.hpp"

namespace Planning {

/**
 * A collection of per-robot trajectories.
 */
class TrajectoryCollection {
public:
    // Per-robot (trajectory, priority)
    using Entry = std::tuple<std::shared_ptr<const Trajectory>, int>;

    std::array<Entry, kNumShells> get() {
        std::lock_guard lock(lock_);
        return robot_trajectories_;
    }

    void put(int robot_id, std::shared_ptr<const Trajectory> trajectory, int priority) {
        std::lock_guard lock(lock_);
        robot_trajectories_.at(robot_id) = std::make_tuple(std::move(trajectory), priority);
    }

private:
    std::mutex lock_;
    std::array<Entry, kNumShells> robot_trajectories_ = {};
};

class PlannerForRobot {
public:
    PlannerForRobot(int robot_id, rclcpp::Node::SharedPtr node,
                    TrajectoryCollection* robot_trajectories);

private:
    PlanRequest make_request(const RobotIntent& intent);
    Trajectory plan_for_robot(const Planning::PlanRequest& request);

    rclcpp::Node::SharedPtr node_;
    std::vector<std::unique_ptr<Planner>> planners_;

    int robot_id_;
    WorldState latest_world_state_;
    TrajectoryCollection* robot_trajectories_;
    rj_geometry::ShapeSet global_obstacles_;
    rj_geometry::ShapeSet goal_zone_obstacles_;
    bool is_goalie_ = false;

    rclcpp::Subscription<WorldState::Msg>::SharedPtr world_state_sub_;
    rclcpp::Subscription<RobotIntent::Msg>::SharedPtr intent_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr goal_zone_obstacles_sub_;
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_sub_;
    rclcpp::Publisher<Trajectory::Msg>::SharedPtr trajectory_pub_;
};

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    std::vector<PlannerForRobot> robots_planners_;
    TrajectoryCollection robot_trajectories_;
};

}  // namespace Planning
