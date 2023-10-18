#pragma once

#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// for ROS actions
#include <rclcpp_action/rclcpp_action.hpp>

#include <context.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/detail/field_dimensions__struct.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/srv/plan_hypothetical_path.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "node.hpp"
#include "planner/path_planner.hpp"
#include "planner/plan_request.hpp"
#include "planning/planner/escape_obstacles_path_planner.hpp"
#include "planning/trajectory_collection.hpp"
#include "planning_params.hpp"
#include "robot_intent.hpp"
#include "trajectory.hpp"
#include "world_state.hpp"

namespace planning {

/**
 * Aggregates info from other ROS nodes into an unchanging "global state" for
 * each planner. Read-only (from PlannerNode's perspective).
 *
 * ("Global state" in quotes since many of these fields can be changed by other
 * nodes; however, to PlannerNode these are immutable.)
 */
class GlobalState {
public:
    GlobalState(rclcpp::Node* node) {
        play_state_sub_ = node->create_subscription<rj_msgs::msg::PlayState>(
            referee::topics::kPlayStateTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::PlayState::SharedPtr state) {  // NOLINT
                last_play_state_ = rj_convert::convert_from_ros(*state);
                set_static_obstacles();
            });
        game_settings_sub_ = node->create_subscription<rj_msgs::msg::GameSettings>(
            config_server::topics::kGameSettingsTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::GameSettings::SharedPtr settings) {  // NOLINT
                last_game_settings_ = rj_convert::convert_from_ros(*settings);
            });
        goalie_sub_ = node->create_subscription<rj_msgs::msg::Goalie>(
            referee::topics::kGoalieTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::Goalie::SharedPtr goalie) {  // NOLINT
                last_goalie_id_ = goalie->goalie_id;
            });
        global_obstacles_sub_ = node->create_subscription<rj_geometry_msgs::msg::ShapeSet>(
            planning::topics::kGlobalObstaclesTopic, rclcpp::QoS(1),
            [this](rj_geometry_msgs::msg::ShapeSet::SharedPtr global_obstacles) {  // NOLINT
                last_global_obstacles_ = rj_convert::convert_from_ros(*global_obstacles);
            });
        world_state_sub_ = node->create_subscription<rj_msgs::msg::WorldState>(
            vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
            [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
                last_world_state_ = rj_convert::convert_from_ros(*world_state);
            });
        coach_state_sub_ = node->create_subscription<rj_msgs::msg::CoachState>(
            "/strategy/coach_state", rclcpp::QoS(1),
            [this](rj_msgs::msg::CoachState::SharedPtr coach_state) {  // NOLINT
                last_coach_state_ = *coach_state;
            });
        field_dimensions_sub_ = node->create_subscription<rj_msgs::msg::FieldDimensions>(
            ::config_server::topics::kFieldDimensionsTopic, 10,
            [this](const rj_msgs::msg::FieldDimensions::SharedPtr msg) {
                current_field_dimensions_ = rj_convert::convert_from_ros(*msg);
                have_field_dimensions_ = true;
            });
    }

    [[nodiscard]] PlayState play_state() const {
        return last_play_state_;
    }
    [[nodiscard]] GameSettings game_settings() const {
        return last_game_settings_;
    }
    [[nodiscard]] int goalie_id() const {
        return last_goalie_id_;
    }
    [[nodiscard]] rj_geometry::ShapeSet global_obstacles() const {
        return last_global_obstacles_;
    }
    [[nodiscard]] rj_geometry::ShapeSet def_area_obstacles() const {
        return last_def_area_obstacles_;
    }
    [[nodiscard]] const WorldState* world_state() const {
        return &last_world_state_;
    }
    [[nodiscard]] const rj_msgs::msg::CoachState coach_state() const {
        return last_coach_state_;
    }

private:
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_sub_;
    rclcpp::Subscription<rj_geometry_msgs::msg::ShapeSet>::SharedPtr def_area_obstacles_sub_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::CoachState>::SharedPtr coach_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::FieldDimensions>::SharedPtr field_dimensions_sub_;

    PlayState last_play_state_ = PlayState::halt();
    FieldDimensions current_field_dimensions_;
    bool have_field_dimensions_ = false;
    GameSettings last_game_settings_;
    int last_goalie_id_;
    rj_geometry::ShapeSet last_global_obstacles_;
    rj_geometry::ShapeSet last_def_area_obstacles_;
    WorldState last_world_state_;
    rj_msgs::msg::CoachState last_coach_state_;

    rj_geometry::ShapeSet create_defense_area_obstacles() {
        // need field dimensions and to be initialized for this to
        // work
        // Create defense areas as rectangular area obstacles
        auto our_defense_area{std::make_shared<rj_geometry::Rect>(
            std::move(current_field_dimensions_.our_defense_area()))};

        // Sometimes there is a greater distance we need to keep:
        // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_robot_too_close_to_opponent_defense_area
        // TODO(sid-parikh): update this conditional. gameplay_node used different set of checks
        // than rules imply
        bool is_extra_dist_necessary = (last_play_state_.state() == PlayState::State::Stop ||
                                        last_play_state_.restart() == PlayState::Restart::Free);

        // Also add a slack around the box
        float slack_around_box{0.3f};

        auto their_defense_area =
            is_extra_dist_necessary
                ? std::make_shared<rj_geometry::Rect>(
                      std::move(current_field_dimensions_.their_defense_area_padded(slack_around_box)))
                : std::make_shared<rj_geometry::Rect>(
                      std::move(current_field_dimensions_.their_defense_area()));

        // Combine both defense areas into ShapeSet
        rj_geometry::ShapeSet def_area_obstacles{};
        def_area_obstacles.add(our_defense_area);
        def_area_obstacles.add(their_defense_area);

        return def_area_obstacles;
    }

    void set_static_obstacles() {
        if (have_field_dimensions_) {
            last_def_area_obstacles_ = create_defense_area_obstacles();
        }
    }
};

/**
 * @brief Handles one robot's planning, which for us is a translation of RobotIntent to
 * Trajectory. Bundles together all relevant ROS pub/subs and forwards
 * RobotIntents to the appropriate PathPlanner (see path_planner.hpp).
 *
 * In general, prefers default behavior and WARN-level logs over crashing.
 *
 * (PlannerNode makes N PlannerForRobots and handles them all, see
 * PlannerNode below.)
 */
class PlannerForRobot {
public:
    PlannerForRobot(int robot_id, rclcpp::Node* node, TrajectoryCollection* robot_trajectories,
                    const GlobalState& global_state);

    PlannerForRobot(PlannerForRobot&&) = delete;
    const PlannerForRobot& operator=(PlannerForRobot&&) = delete;
    PlannerForRobot(const PlannerForRobot&) = delete;
    const PlannerForRobot& operator=(const PlannerForRobot&) = delete;

    ~PlannerForRobot() = default;

    /**
     * Entry point for the planner node's ActionServer.
     *
     * Creates and publishes a Trajectory based on the given RobotIntent. Also
     * publishes a ManipulatorSetpoint to control kicker/dribbler/chipper.
     */
    void execute_intent(const RobotIntent& intent);

    /*
     * @brief estimate the amount of time it would take for a robot to execute a robot intent
     * (SERVICE).
     *
     * @param request Requested RobotIntent resulting in the hypothetical robot path.
     * @param response The response object that will contain the resultant time to completion of a
     * hypothetical path.
     */
    // TODO(Kevin): I broke this, sorry
    void plan_hypothetical_robot_path(
        const std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Request>& request,
        std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Response>& response);

    /**
     * @return RJ::Seconds time left for the trajectory to complete
     *
     * Note: RJ::Seconds is an alias for std::chrono::duration<double>.
     */
    [[nodiscard]] std::optional<RJ::Seconds> get_time_left() const;

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
     * @brief Get a Trajectory based on the string name given in MotionCommand.
     * Guaranteed to output a valid Trajectory: defaults to
     * EscapeObstaclesPathPlanner if requested planner fails, and gives WARN
     * logs.
     *
     * @details Uses unsafe_plan_for_robot() to get a plan, handling any
     * Exceptions that come up.
     *
     * @param request PlanRequest to create a Trajectory from
     *
     * @return Trajectory (timestamped series of poses & twists) that satisfies
     * the PlanRequest as well as possible
     */
    Trajectory safe_plan_for_robot(const planning::PlanRequest& request);

    /*
     * @brief Get a Trajectory based on the string name given in MotionCommand.
     *
     * @details Differs from safe_plan_for_robot() in that it will throw Exceptions if
     * planners fail (which safe_plan_for_robot() handles).
     *
     * @param request PlanRequest to create a Trajectory from
     *
     * @return Trajectory (timestamped series of poses & twists) that satisfies
     * the PlanRequest as well as possible
     */
    Trajectory unsafe_plan_for_robot(const planning::PlanRequest& request);

    /*
     * @brief Check that robot is visible in world_state and that world_state has been
     * updated recently.
     */
    [[nodiscard]] bool robot_alive() const;

    rclcpp::Node* node_;

    // unique_ptrs here because we don't want to transfer ownership of
    // thePathPlanner objects anywhere else
    // (must be some type of ptr for polymorphism)
    std::unordered_map<std::string, std::unique_ptr<PathPlanner>> path_planners_;
    // EscapeObstaclesPathPlanner is our default path planner
    // because when robots start inside of an obstacle, all other planners will fail
    // TODO(Kevin): make EscapeObstaclesPathPlanner default start of all planner FSM so this doesn't
    // happen
    std::unique_ptr<PathPlanner> default_path_planner_{
        std::make_unique<EscapeObstaclesPathPlanner>()};

    // raw ptr here because current_path_planner_ should not take ownership
    // from any of the unique_ptrs to PathPlanners
    PathPlanner* current_path_planner_{default_path_planner_.get()};

    int robot_id_;
    TrajectoryCollection* robot_trajectories_;
    const GlobalState& global_state_;

    bool had_break_beam_ = false;

    rclcpp::Subscription<RobotIntent::Msg>::SharedPtr intent_sub_;
    rclcpp::Subscription<rj_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Publisher<Trajectory::Msg>::SharedPtr trajectory_topic_;
    rclcpp::Publisher<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr manipulator_pub_;
    rclcpp::Service<rj_msgs::srv::PlanHypotheticalPath>::SharedPtr hypothetical_path_service_;

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
    std::vector<std::unique_ptr<PlannerForRobot>> robot_planners_;
    TrajectoryCollection robot_trajectories_;
    GlobalState global_state_;
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

    /*
     * @brief Track the current state of a robot's task. This is how
     * PlannerNode ensures each robot only has one task running.
     */
    struct ServerTaskState {
        ServerTaskState() = default;
        ~ServerTaskState() = default;
        // disallow copy/move operators
        ServerTaskState(const ServerTaskState& state) = delete;
        ServerTaskState& operator=(const ServerTaskState& state) = delete;
        ServerTaskState(const ServerTaskState&& state) = delete;
        ServerTaskState& operator=(const ServerTaskState&& state) = delete;

        volatile std::atomic_bool is_executing{false};
        volatile std::atomic_bool new_task_waiting_signal{false};
    };

    // create an array, kNumShells long, of ServerTaskState structs for
    // PlannerNode to use
    std::array<ServerTaskState, kNumShells> server_task_states_;
};

}  // namespace planning
