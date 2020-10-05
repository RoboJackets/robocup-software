#include "planner_node.hpp"

#include <rj_constants/topic_names.hpp>

#include "instant.hpp"
#include "planning/planner/collect_planner.hpp"
#include "planning/planner/escape_obstacles_path_planner.hpp"
#include "planning/planner/line_kick_planner.hpp"
#include "planning/planner/path_target_planner.hpp"
#include "planning/planner/pivot_path_planner.hpp"
#include "planning/planner/settle_planner.hpp"
#include "robot.hpp"

namespace Planning {

PlannerNode::PlannerNode() : rclcpp::Node("planner") {
    robots_planners_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        robots_planners_.emplace_back(i, create_sub_node("planner_robot_" + std::to_string(i)), &robot_trajectories_);
    }
}

PlannerForRobot::PlannerForRobot(int robot_id,
                                 rclcpp::Node::SharedPtr node,
                                 TrajectoryCollection* robot_trajectories)
    : node_{std::move(node)},
      robot_id_{robot_id},
      robot_trajectories_{robot_trajectories} {
    planners_.push_back(std::make_unique<PathTargetPlanner>());
    planners_.push_back(std::make_unique<SettlePlanner>());
    planners_.push_back(std::make_unique<CollectPlanner>());
    planners_.push_back(std::make_unique<LineKickPlanner>());
    planners_.push_back(std::make_unique<PivotPathPlanner>());

    // The empty planner should always be last.
    planners_.push_back(std::make_unique<EscapeObstaclesPathPlanner>());

    // Set up ROS
    trajectory_pub_ = node_->create_publisher<Trajectory::Msg>(
        planning::topics::trajectory_pub(robot_id), rclcpp::QoS(1).transient_local());
    intent_sub_ = node_->create_subscription<RobotIntent::Msg>(
        gameplay::topics::robot_intent_pub(robot_id),
        rclcpp::QoS(1),
        [this](RobotIntent::Msg::SharedPtr intent) { // NOLINT
          auto plan_request = make_request(rj_convert::convert_from_ros(*intent));
          auto trajectory = plan_for_robot(plan_request);
          trajectory_pub_->publish(
              rj_convert::convert_to_ros(trajectory));
          robot_trajectories_->put(robot_id_, std::make_shared<Trajectory>(std::move(trajectory)), intent->priority);
        });
    world_state_sub_ = node_->create_subscription<WorldState::Msg>(
        vision_filter::topics::kWorldStatePub,
        rclcpp::QoS(1).transient_local(),
        [this](WorldState::Msg::SharedPtr state) { // NOLINT
          latest_world_state_ = rj_convert::convert_from_ros(*state);
        });
    global_obstacles_sub_ = node_->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
        planning::topics::kGlobalObstaclesPub,
        rclcpp::QoS(1).transient_local(),
        [this](rj_geometry::ShapeSet::Msg::SharedPtr global_obstacles) { // NOLINT
          global_obstacles_ = rj_convert::convert_from_ros(*global_obstacles);
        });
    goal_zone_obstacles_sub_ = node_->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
        planning::topics::kGoalZoneObstacles,
        rclcpp::QoS(1).transient_local(),
        [this](rj_geometry::ShapeSet::Msg::SharedPtr goal_zone_obstacles) { // NOLINT
          goal_zone_obstacles_ = rj_convert::convert_from_ros(*goal_zone_obstacles);
        });
    goalie_sub_ = node_->create_subscription<rj_msgs::msg::Goalie>(
        planning::topics::kGoalZoneObstacles,
        rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::Goalie::SharedPtr goalie) { // NOLINT
          is_goalie_ = (robot_id_ == goalie->goalie_id);
        });
}

PlanRequest PlannerForRobot::make_request(const RobotIntent& intent) {
    const auto& robot = latest_world_state_.our_robots.at(robot_id_);
    const auto& start = RobotInstant{
        robot.pose,
        robot.velocity,
        robot.timestamp
    };
    rj_geometry::ShapeSet obstacles = global_obstacles_;
    if (!is_goalie_) {
        obstacles.add(goal_zone_obstacles_);
    }

    const auto robot_trajectories_hold = robot_trajectories_->get();
    std::array<const Trajectory*, kNumShells> planned_trajectories = {};

    for (int i = 0; i < kNumShells; i++) {
        const auto& [trajectory, priority] = robot_trajectories_hold.at(i);
        if (i != robot_id_ && priority >= intent.priority) {
            planned_trajectories.at(i) = trajectory.get();
        }
    }

    // TODO(Kyle): Implement debug drawing in ROS
    DebugDrawer* debug_drawer = nullptr;

    return PlanRequest{start,
                       intent.motion_command,
                       RobotConstraints(),
                       global_obstacles_,
                       intent.local_obstacles,
                       planned_trajectories,
                       static_cast<unsigned int>(robot_id_),
                       &latest_world_state_,
                       intent.priority,
                       debug_drawer};
}

Trajectory PlannerForRobot::plan_for_robot(const Planning::PlanRequest& request) {
    // Try each planner in sequence until we find one that is applicable.
    // This gives the planners a sort of "priority" - this makes sense, because
    // the empty planner is always last.
    Trajectory trajectory;
    for (auto& planner : planners_) {
        // If this planner could possibly plan for this command, try to make
        // a plan.
        if (trajectory.empty() && planner->is_applicable(request.motion_command)) {
            RobotInstant start_instant = request.start;
            trajectory = planner->plan(request);
        }

        // If it fails, or if the planner was not used, the trajectory will
        // still be empty. Reset the planner.
        if (trajectory.empty()) {
            planner->reset();
        } else {
            if (!trajectory.angles_valid()) {
                throw std::runtime_error("Trajectory returned from " + planner->name() +
                                         " has no angle profile!");
            }

            if (!trajectory.time_created().has_value()) {
                throw std::runtime_error("Trajectory returned from " + planner->name() +
                                         " has no timestamp!");
            }
        }
    }

    if (trajectory.empty()) {
        SPDLOG_ERROR("No valid planner! Did you forget to specify a default planner?");
        trajectory = Trajectory{{request.start}};
        trajectory.set_debug_text("Error: No Valid Planners");
    }

    return trajectory;
}

}  // namespace Planning
