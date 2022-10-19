#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
// for ROS actions
#include <rclcpp_action/rclcpp_action.hpp>

#include <context.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "node.hpp"
#include "planner/plan_request.hpp"
#include "planner/planner.hpp"
#include "planning_params.hpp"
#include "robot_intent.hpp"
#include "trajectory.hpp"
#include "world_state.hpp"

namespace planning {

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
        // associate a (Trajectory, priority) tuple with a robot id
        std::lock_guard lock(lock_);
        robot_trajectories_.at(robot_id) = std::make_tuple(std::move(trajectory), priority);
    }

    std::shared_ptr<const Trajectory> get(int robot_id) {
        // return the most recent Trajectory associated with a robot id
        // TODO(Kevin): return priority?
        std::lock_guard lock(lock_);
        auto traj_tuple = robot_trajectories_.at(robot_id);
        return std::get<0>(traj_tuple);
    }

private:
    std::mutex lock_;
    std::array<Entry, kNumShells> robot_trajectories_ = {};
};

class SharedStateInfo {
public:
    SharedStateInfo(rclcpp::Node* node) {
        play_state_sub_ = node->create_subscription<rj_msgs::msg::PlayState>(
            referee::topics::kPlayStatePub, rclcpp::QoS(1),
            [this](rj_msgs::msg::PlayState::SharedPtr state) {  // NOLINT
                auto lock = std::lock_guard(mutex_);
                last_play_state_ = rj_convert::convert_from_ros(*state);
            });
        game_settings_sub_ = node->create_subscription<rj_msgs::msg::GameSettings>(
            config_server::topics::kGameSettingsPub, rclcpp::QoS(1),
            [this](rj_msgs::msg::GameSettings::SharedPtr settings) {  // NOLINT
                auto lock = std::lock_guard(mutex_);
                last_game_settings_ = rj_convert::convert_from_ros(*settings);
            });
        goalie_sub_ = node->create_subscription<rj_msgs::msg::Goalie>(
            referee::topics::kGoaliePub, rclcpp::QoS(1),
            [this](rj_msgs::msg::Goalie::SharedPtr goalie) {  // NOLINT
                auto lock = std::lock_guard(mutex_);
                last_goalie_id_ = goalie->goalie_id;
            });
        global_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
            planning::topics::kGlobalObstaclesPub, rclcpp::QoS(1),
            [this](rj_geometry_msgs::msg::ShapeSet::SharedPtr global_obstacles) {  // NOLINT
                auto lock = std::lock_guard(mutex_);
                last_global_obstacles_ = rj_convert::convert_from_ros(*global_obstacles);
            });
        def_area_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
            planning::topics::kDefAreaObstaclesPub, rclcpp::QoS(1),
            [this](rj_geometry_msgs::msg::ShapeSet::SharedPtr def_area_obstacles) {  // NOLINT
                auto lock = std::lock_guard(mutex_);
                last_def_area_obstacles_ = rj_convert::convert_from_ros(*def_area_obstacles);
            });
        world_state_sub_ = node->create_subscription<rj_msgs::msg::WorldState>(
            vision_filter::topics::kWorldStatePub, rclcpp::QoS(1),
            [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
                auto lock = std::lock_guard(mutex_);
                last_world_state_ = rj_convert::convert_from_ros(*world_state);
            });
    }

    [[nodiscard]] PlayState play_state() const {
        auto lock = std::lock_guard(mutex_);
        return last_play_state_;
    }
    [[nodiscard]] GameSettings game_settings() const {
        auto lock = std::lock_guard(mutex_);
        return last_game_settings_;
    }
    [[nodiscard]] int goalie_id() const {
        auto lock = std::lock_guard(mutex_);
        return last_goalie_id_;
    }
    [[nodiscard]] rj_geometry::ShapeSet global_obstacles() const {
        auto lock = std::lock_guard(mutex_);
        return last_global_obstacles_;
    }
    [[nodiscard]] rj_geometry::ShapeSet def_area_obstacles() const {
        auto lock = std::lock_guard(mutex_);
        return last_def_area_obstacles_;
    }
    [[nodiscard]] const WorldState* world_state() const {
        auto lock = std::lock_guard(mutex_);
        return &last_world_state_;
    }

private:
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr def_area_obstacles_sub_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;

    mutable std::mutex mutex_;
    PlayState last_play_state_ = PlayState::halt();
    GameSettings last_game_settings_;
    int last_goalie_id_;
    rj_geometry::ShapeSet last_global_obstacles_;
    rj_geometry::ShapeSet last_def_area_obstacles_;
    WorldState last_world_state_;
};

/**
 * Interface for one robot's planning, which for us is a RobotIntent to Trajectory
 * translation. Planner node makes N PlannerForRobots and handles them all.
 */
class PlannerForRobot {
public:
    PlannerForRobot(int robot_id, rclcpp::Node* node, TrajectoryCollection* robot_trajectories,
                    SharedStateInfo* shared_state);

    PlannerForRobot(PlannerForRobot&&) = delete;
    const PlannerForRobot& operator=(PlannerForRobot&&) = delete;
    PlannerForRobot(const PlannerForRobot&) = delete;
    const PlannerForRobot& operator=(const PlannerForRobot&) = delete;

    ~PlannerForRobot() = default;

    /**
     * Entry point for the planner node's ActionServer.
     *
     * Creates and publishes a Trajectory based on the given RobotIntent.
     */
    void execute_trajectory(const RobotIntent& intent);

    /**
     * @return RJ::Seconds time left for the trajectory to complete
     *
     * Note: RJ::Seconds is an alias for std::chrono::duration<double>.
     */
    RJ::Seconds get_time_left() const;

    /*
     * @return true if current planner is done, false otherwise.
     */
    [[nodiscard]] bool is_done() const;

private:
    /**
     * @brief Create a PlanRequest based on the given RobotIntent.
     *
     * @details This is how gameplay's RobotIntents end up in the right planner (e.g.
     * how a Pivot skill goes to PivotPath planner).
     *
     * @param intent RobotIntent msg
     *
     * @return PlanRequest based on input RobotIntent
     */
    PlanRequest make_request(const RobotIntent& intent);

    /*
     * @brief Get a Trajectory based on the PlanRequest by ticking through all
     * available planners.
     *
     * @details (Each planner implements an "is_applicable(PlanRequest.motion_command)").
     *
     * @param request PlanRequest that needs a motion plan made
     *
     * @return Trajectory (timestamped series of poses & twists) that satisfies
     * the PlanRequest as well as possible
     */
    Trajectory plan_for_robot(const planning::PlanRequest& request);

    /*
     * @brief Check that robot is visible in world_state and that world_state has been
     * updated recently.
     */
    [[nodiscard]] bool robot_alive() const;

    rclcpp::Node* node_;
    std::vector<std::shared_ptr<Planner>> planners_;
    std::shared_ptr<Planner> current_planner_;

    int robot_id_;
    TrajectoryCollection* robot_trajectories_;
    SharedStateInfo* shared_state_;

    bool had_break_beam_ = false;

    rclcpp::Subscription<RobotIntent::Msg>::SharedPtr intent_sub_;
    rclcpp::Subscription<rj_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<Trajectory::Msg>::SharedPtr trajectory_pub_;

    rj_drawing::RosDebugDrawer debug_draw_;
};

/**
 * ROS node that spawns many PlannerForRobots and helps coordinate them.
 */
class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

    using RobotMove = rj_msgs::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

private:
    std::vector<std::shared_ptr<PlannerForRobot>> robots_planners_;
    TrajectoryCollection robot_trajectories_;
    SharedStateInfo shared_state_;
    ::params::LocalROS2ParamProvider param_provider_;

    // setup ActionServer for RobotMove.action
    // follows the standard AS protocol, see ROS2 docs & RobotMove.action
    rclcpp_action::Server<RobotMove>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const RobotMove::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleRobotMove> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleRobotMove> goal_handle);

    /*
     * @brief Upon being given a RobotIntent, publish an appropriate
     * Trajectory, send time remaining as feedback, and return success when
     * done. Blocking (as in, will loop until complete).
     */
    void execute(const std::shared_ptr<GoalHandleRobotMove> goal_handle);
};

}  // namespace planning
