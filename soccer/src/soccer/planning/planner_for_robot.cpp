#include "planner_for_robot.hpp"

#include "game_state.hpp"
#include "planning/planner/collect_path_planner.hpp"
#include "planning/planner/escape_obstacles_path_planner.hpp"
#include "planning/planner/goalie_idle_path_planner.hpp"
#include "planning/planner/intercept_path_planner.hpp"
#include "planning/planner/line_kick_path_planner.hpp"
#include "planning/planner/line_kick_planner_one.hpp"
#include "planning/planner/line_kick_planner_two.hpp"
#include "planning/planner/path_target_path_planner.hpp"
#include "planning/planner/pivot_path_planner.hpp"
#include "planning/planner/settle_path_planner.hpp"

namespace planning {

PlannerForRobot::PlannerForRobot(int robot_id, rclcpp::Node* node,
                                 TrajectoryCollection* robot_trajectories,
                                 const GlobalState& global_state)
    : node_{node},
      robot_id_{robot_id},
      robot_trajectories_{robot_trajectories},
      global_state_{global_state},
      debug_draw_{
          node->create_publisher<rj_drawing_msgs::msg::DebugDraw>(viz::topics::kDebugDrawTopic, 10),
          fmt::format("planning_{}", robot_id)} {
    // create map of {planner name -> planner}
    path_planners_[GoalieIdlePathPlanner().name()] = std::make_unique<GoalieIdlePathPlanner>();
    path_planners_[InterceptPathPlanner().name()] = std::make_unique<InterceptPathPlanner>();
    path_planners_[PathTargetPathPlanner().name()] = std::make_unique<PathTargetPathPlanner>();
    path_planners_[SettlePathPlanner().name()] = std::make_unique<SettlePathPlanner>();
    path_planners_[CollectPathPlanner().name()] = std::make_unique<CollectPathPlanner>();
    path_planners_[LineKickPathPlanner().name()] = std::make_unique<LineKickPathPlanner>();
    path_planners_[LineKickPlannerOne().name()] = std::make_unique<LineKickPlannerOne>();
    path_planners_[LineKickPlannerTwo().name()] = std::make_unique<LineKickPlannerTwo>();
    path_planners_[PivotPathPlanner().name()] = std::make_unique<PivotPathPlanner>();
    path_planners_[EscapeObstaclesPathPlanner().name()] =
        std::make_unique<EscapeObstaclesPathPlanner>();

    // publish paths to control
    trajectory_topic_ = node_->create_publisher<Trajectory::Msg>(
        planning::topics::trajectory_topic(robot_id), rclcpp::QoS(1).transient_local());

    // publish kicker/dribbler cmds directly to radio
    manipulator_pub_ = node->create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
        control::topics::manipulator_setpoint_topic(robot_id), rclcpp::QoS(10));

    // for ball sense and possession
    robot_status_sub_ = node_->create_subscription<rj_msgs::msg::RobotStatus>(
        radio::topics::robot_status_topic(robot_id), rclcpp::QoS(1),
        [this](rj_msgs::msg::RobotStatus::SharedPtr status) {  // NOLINT
            had_break_beam_ = status->has_ball_sense;
        });

    // For hypothetical path planning
    hypothetical_path_service_ = node_->create_service<rj_msgs::srv::PlanHypotheticalPath>(
        fmt::format("hypothetical_trajectory_robot_{}", robot_id),
        [this](const std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Request> request,
               std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Response> response) {
            plan_hypothetical_robot_path(request, response);
        });
}

void PlannerForRobot::execute_intent(const RobotIntent& intent) {
    if (robot_alive()) {
        // plan a path and send it to control
        auto plan_request = make_request(intent);

        auto trajectory = safe_plan_for_robot(plan_request);
        trajectory_topic_->publish(rj_convert::convert_to_ros(trajectory));

        // send the kick/dribble commands to the radio
        manipulator_pub_->publish(rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
                                      .shoot_mode(intent.shoot_mode)
                                      .trigger_mode(intent.trigger_mode)
                                      .kick_speed(intent.kick_speed)
                                      .dribbler_speed(plan_request.dribbler_speed));

        /*
        // TODO (PR #1970): fix TrajectoryCollection
        // store all latest trajectories in a mutex-locked shared map
        robot_trajectories_->put(robot_id_, std::make_shared<Trajectory>(std::move(trajectory)),
                                 intent.priority);
        */
    }
}

void PlannerForRobot::plan_hypothetical_robot_path(
    const std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Request>& request,
    std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Response>& response) {
    /* const auto intent = rj_convert::convert_from_ros(request->intent); */
    /* auto plan_request = make_request(intent); */
    /* auto trajectory = safe_plan_for_robot(plan_request); */
    /* RJ::Seconds trajectory_duration = trajectory.duration(); */
    /* response->estimate = rj_convert::convert_to_ros(trajectory_duration); */
}

std::optional<RJ::Seconds> PlannerForRobot::get_time_left() const {
    // TODO(p-nayak): why does this say 3s even when the robot is on its point?
    // get the Traj out of the relevant [Trajectory, priority] tuple in
    // robot_trajectories_

    /*
    // TODO (PR #1970): fix TrajectoryCollection
    const auto& [latest_traj, priority] = robot_trajectories_->get(robot_id_);
    if (!latest_traj) {
        return std::nullopt;
    }
    return latest_traj->end_time() - RJ::now();
    */
    return std::nullopt;
}

PlanRequest PlannerForRobot::make_request(const RobotIntent& intent) {
    // pass global_state_ directly
    const auto* world_state = global_state_.world_state();
    const auto goalie_id = global_state_.goalie_id();
    const auto play_state = global_state_.play_state();

    float min_dist_from_ball{};
    float max_robot_speed{};
    float max_dribbler_speed{};

    // Global Overrides
    switch (play_state.state()) {
        case PlayState::State::Halt:
            min_dist_from_ball = 0;
            max_robot_speed = 0;
            max_dribbler_speed = 0;
            break;
        case PlayState::State::Stop:
            min_dist_from_ball = 0.5;
            max_robot_speed = 1.5;
            max_dribbler_speed = 0;
            break;
        case PlayState::State::Playing:
        default:

            min_dist_from_ball = 0;
            // Unbounded speed. Setting to -1 or 0 crashes planner, so use large
            // number instead.
            max_robot_speed = 10.0;
            max_dribbler_speed = 255;
            break;
    }

    // publish new necessary information

    const auto& robot = world_state->our_robots.at(robot_id_);
    const auto start = RobotInstant{robot.pose, robot.velocity, robot.timestamp};

    const auto global_obstacles = global_state_.global_obstacles();
    rj_geometry::ShapeSet real_obstacles = global_obstacles;

    const auto def_area_obstacles = global_state_.def_area_obstacles();
    rj_geometry::ShapeSet virtual_obstacles = intent.local_obstacles;
    const bool is_goalie = goalie_id == robot_id_;
    if (!is_goalie) {
        virtual_obstacles.add(def_area_obstacles);
    }

    /*
    // TODO (PR #1970): fix TrajectoryCollection
    // make a copy instead of getting the actual shared_ptr to Trajectory
    std::array<std::optional<Trajectory>, kNumShells> planned_trajectories;

    for (size_t i = 0; i < kNumShells; i++) {
        // TODO(Kevin): check that priority works (seems like
        // robot_trajectories_ is passed on init, when no planning has occured
        // yet)
        const auto& [trajectory, priority] = robot_trajectories_->get(i);
        if (i != robot_id_ && priority >= intent.priority) {
            if (!trajectory) {
                planned_trajectories[i] = std::nullopt;
            } else {
                planned_trajectories[i] = std::make_optional<const
    Trajectory>(*trajectory.get());
            }
        }
    }
    */

    RobotConstraints constraints;
    MotionCommand motion_command;
    // Attempting to create trajectories with max speeds <= 0 crashes the planner (during RRT
    // generation)
    if (max_robot_speed == 0.0f) {
        // If override speed is set to 0,
        // force HALT by replacing the MotionCommand with an empty one.
        motion_command = MotionCommand{};
    } else if (max_robot_speed < 0.0f) {
        // If override speed set to negative, assume infinity.
        // Negative numbers cause crashes, but 10 m/s is an effectively infinite limit.
        motion_command = intent.motion_command;
        constraints.mot.max_speed = 10.0f;
    } else {
        motion_command = intent.motion_command;
        constraints.mot.max_speed = max_robot_speed;
    }

    float dribble_speed =
        std::min(static_cast<float>(intent.dribbler_speed), static_cast<float>(max_dribbler_speed));

    return PlanRequest{start,
                       motion_command,
                       constraints,
                       std::move(real_obstacles),
                       std::move(virtual_obstacles),
                       robot_trajectories_,
                       static_cast<unsigned int>(robot_id_),
                       world_state,
                       play_state,
                       intent.priority,
                       &debug_draw_,
                       had_break_beam_,
                       min_dist_from_ball,
                       dribble_speed};
}

Trajectory PlannerForRobot::unsafe_plan_for_robot(const planning::PlanRequest& request) {
    if (path_planners_.count(request.motion_command.name) == 0) {
        throw std::runtime_error(fmt::format("ID {}: MotionCommand name <{}> does not exist!",
                                             robot_id_, request.motion_command.name));
    }

    // get Trajectory from the planner requested in MotionCommand
    current_path_planner_ = path_planners_[request.motion_command.name].get();
    Trajectory trajectory = current_path_planner_->plan(request);

    if (trajectory.empty()) {
        // empty Trajectory means current_path_planner_ has failed
        // if current_path_planner_ fails, reset it before throwing exception
        current_path_planner_->reset();
        throw std::runtime_error(fmt::format("PathPlanner <{}> failed to create valid Trajectory!",
                                             current_path_planner_->name()));
    }

    if (!trajectory.angles_valid()) {
        throw std::runtime_error(fmt::format("Trajectory returned from <{}> has no angle profile!",
                                             current_path_planner_->name()));
    }

    if (!trajectory.time_created().has_value()) {
        throw std::runtime_error(fmt::format("Trajectory returned from <{}> has no timestamp!",
                                             current_path_planner_->name()));
    }

    return trajectory;
}

Trajectory PlannerForRobot::safe_plan_for_robot(const planning::PlanRequest& request) {
    Trajectory trajectory;
    try {
        trajectory = unsafe_plan_for_robot(request);
    } catch (std::runtime_error exception) {
        // SPDLOG_WARN("PlannerForRobot {} error caught: {}", robot_id_, exception.what());
        // SPDLOG_WARN("PlannerForRobot {}: Defaulting to EscapeObstaclesPathPlanner", robot_id_);

        current_path_planner_ = default_path_planner_.get();
        trajectory = current_path_planner_->plan(request);
        // TODO(Kevin): planning should be able to send empty Trajectory
        // without crashing, instead of resorting to default planner
        // (currently the ros_convert throws "cannot serialize trajectory with
        // invalid angles")
    }

    // draw robot's desired path
    std::vector<rj_geometry::Point> path;
    std::transform(trajectory.instants().begin(), trajectory.instants().end(),
                   std::back_inserter(path),
                   [](const auto& instant) { return instant.position(); });
    debug_draw_.draw_path(path);

    // draw robot's desired endpoint
    debug_draw_.draw_circle(rj_geometry::Circle(path.back(), kRobotRadius), Qt::black);

    // draw obstacles for this robot
    // TODO: these will stack atop each other, since each robot draws obstacles
    debug_draw_.draw_shapes(global_state_.global_obstacles(), QColor(255, 0, 0, 30));
    debug_draw_.draw_shapes(request.virtual_obstacles, QColor(255, 0, 0, 30));
    debug_draw_.publish();

    return trajectory;
}

bool PlannerForRobot::robot_alive() const {
    return global_state_.world_state()->our_robots.at(robot_id_).visible &&
           RJ::now() < global_state_.world_state()->last_updated_time + RJ::Seconds(PARAM_timeout);
}

bool PlannerForRobot::is_done() const {
    // no segfaults
    if (current_path_planner_ == nullptr) {
        return false;
    }

    return current_path_planner_->is_done();
}

}  // namespace planning
