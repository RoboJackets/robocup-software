#include "planner_node.hpp"

#include <spdlog/spdlog.h>

#include <rj_constants/topic_names.hpp>
#include <ros_debug_drawer.hpp>

#include "instant.hpp"
#include "planning/planner/collect_planner.hpp"
#include "planning/planner/escape_obstacles_path_planner.hpp"
#include "planning/planner/line_kick_planner.hpp"
#include "planning/planner/path_target_planner.hpp"
#include "planning/planner/pivot_path_planner.hpp"
#include "planning/planner/settle_planner.hpp"

namespace planning {

using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

PlannerNode::PlannerNode()
    : rclcpp::Node("planner", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      shared_state_(this),
      param_provider_{this, kPlanningParamModule} {
    // for _1, _2 etc. below
    using namespace std::placeholders;

    // set up ActionServer + callbacks
    this->action_server_ = rclcpp_action::create_server<RobotMove>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(), "robot_move",
        std::bind(&PlannerNode::handle_goal, this, _1, _2),
        std::bind(&PlannerNode::handle_cancel, this, _1),
        std::bind(&PlannerNode::handle_accepted, this, _1));

    // set up PlannerForRobot objects
    robots_planners_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        auto planner =
            std::make_shared<PlannerForRobot>(i, this, &robot_trajectories_, &shared_state_);
        robots_planners_.emplace_back(std::move(planner));
    }
}

rclcpp_action::GoalResponse PlannerNode::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const RobotMove::Goal> goal) {
    (void)uuid;
    auto delay = std::chrono::milliseconds(1000 / 60);
    rclcpp::Rate loop_rate(delay);

    // TODO(p-nayak): REJECT duplicate goal requests so we aren't constantly replanning them

    // planning::MotionCommand motion_command_ = goal->robot_intent.motion_command;

    // On a new goal, get the current task for the given robot ID and kill it,
    // then set the current task to the new goal. The complexity here is from
    // the shared server_task_states_, which could be modified by any thread
    // running handle_goal(). See PR #1958.
    int robot_id = goal->robot_intent.robot_id;
    auto& robot_task = server_task_states_.at(robot_id);
    auto& is_executing = robot_task.is_executing;
    auto& new_task_waiting_signal = robot_task.new_task_waiting_signal;
    while (is_executing) {
        new_task_waiting_signal = true;
        loop_rate.sleep();
    }
    new_task_waiting_signal = false;
    is_executing = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handle_cancel(
    const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    (void)goal_handle;
    std::shared_ptr<const RobotMove::Goal> goal = goal_handle->get_goal();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handle_accepted(const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // execute() will block (loop) until completion (either success or canceled by client)
    using namespace std::placeholders;
    std::thread{std::bind(&PlannerNode::execute, this, _1), goal_handle}.detach();
}

void PlannerNode::execute(const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    // TODO(Kevin): rate-limit loop to whatever hz planning is limited to
    auto delay = std::chrono::milliseconds(1000 / 60);
    rclcpp::Rate loop_rate(delay);

    // create ptrs to Goal, Result objects per ActionServer API
    std::shared_ptr<const RobotMove::Goal> goal = goal_handle->get_goal();
    std::shared_ptr<RobotMove::Result> result = std::make_shared<RobotMove::Result>();

    // get correct PlannerForRobot object for this robot_id
    int robot_id = goal->robot_intent.robot_id;
    std::shared_ptr<PlannerForRobot> my_robot_planner = robots_planners_[robot_id];

    auto& robot_task = server_task_states_.at(robot_id);

    // loop until goal is done (SUCCEEDED or CANCELED)
    for (;;) {
        auto& new_task_ready = robot_task.new_task_waiting_signal;
        // check if there is a new goal
        if (new_task_ready) {
            result->is_done = false;
            goal_handle->abort(result);
            break;
        }

        // if the ActionClient is trying to cancel the goal, cancel it & terminate early
        if (goal_handle->is_canceling()) {
            result->is_done = false;
            goal_handle->canceled(result);
            break;
        }

        // pub Trajectory based on the RobotIntent
        my_robot_planner->execute_trajectory(rj_convert::convert_from_ros(goal->robot_intent));

        /*
        // TODO (PR #1970): fix TrajectoryCollection
        // send feedback
        std::shared_ptr<RobotMove::Feedback> feedback = std::make_shared<RobotMove::Feedback>();
        if (auto time_left = my_robot_planner->get_time_left()) {
            feedback->time_left = rj_convert::convert_to_ros(time_left.value());
            goal_handle->publish_feedback(feedback);
        }
        */

        // when done, tell client goal is done, break loop
        // TODO(p-nayak): when done, publish empty motion command to this robot's trajectory
        if (my_robot_planner->is_done()) {
            if (rclcpp::ok()) {
                result->is_done = true;
                goal_handle->succeed(result);
                break;
            }
        }
        loop_rate.sleep();
    }
    robot_task.is_executing = false;
}

PlannerForRobot::PlannerForRobot(int robot_id, rclcpp::Node* node,
                                 TrajectoryCollection* robot_trajectories,
                                 SharedStateInfo* shared_state)
    : node_{node},
      robot_id_{robot_id},
      robot_trajectories_{robot_trajectories},
      shared_state_{shared_state},
      debug_draw_{
          node->create_publisher<rj_drawing_msgs::msg::DebugDraw>(viz::topics::kDebugDrawPub, 10),
          fmt::format("planning_{}", robot_id)} {
    planners_.push_back(std::make_shared<PathTargetPlanner>());
    planners_.push_back(std::make_shared<SettlePlanner>());
    planners_.push_back(std::make_shared<CollectPlanner>());
    planners_.push_back(std::make_shared<LineKickPlanner>());
    planners_.push_back(std::make_shared<PivotPathPlanner>());

    // The empty planner should always be last.
    planners_.push_back(std::make_shared<EscapeObstaclesPathPlanner>());

    trajectory_pub_ = node_->create_publisher<Trajectory::Msg>(
        planning::topics::trajectory_pub(robot_id), rclcpp::QoS(1).transient_local());

    // for ball sense and possession
    robot_status_sub_ = node_->create_subscription<rj_msgs::msg::RobotStatus>(
        radio::topics::robot_status_pub(robot_id), rclcpp::QoS(1),
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

void PlannerForRobot::execute_trajectory(const RobotIntent& intent) {
    if (robot_alive()) {
        auto plan_request = make_request(intent);
        auto trajectory = plan_for_robot(plan_request);
        trajectory_pub_->publish(rj_convert::convert_to_ros(trajectory));
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
    const auto intent = rj_convert::convert_from_ros(request->intent);
    auto plan_request = make_request(intent);
    auto trajectory = plan_for_robot(plan_request);
    RJ::Seconds trajectory_duration = trajectory.duration();
    response->estimate = rj_convert::convert_to_ros(trajectory_duration);
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
    const auto* world_state = shared_state_->world_state();
    const auto global_obstacles = shared_state_->global_obstacles();
    const auto def_area_obstacles = shared_state_->def_area_obstacles();
    const auto goalie_id = shared_state_->goalie_id();
    const auto play_state = shared_state_->play_state();
    const bool is_goalie = goalie_id == robot_id_;

    const auto& robot = world_state->our_robots.at(robot_id_);
    const auto start = RobotInstant{robot.pose, robot.velocity, robot.timestamp};
    rj_geometry::ShapeSet real_obstacles = global_obstacles;
    rj_geometry::ShapeSet virtual_obstacles = intent.local_obstacles;
    if (!is_goalie) {
        virtual_obstacles.add(def_area_obstacles);
    }

    /*
    // TODO (PR #1970): fix TrajectoryCollection
    // make a copy instead of getting the actual shared_ptr to Trajectory
    std::array<std::optional<Trajectory>, kNumShells> planned_trajectories;

    for (int i = 0; i < kNumShells; i++) {
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

    // TODO(Kyle): Send constraints from gameplay
    RobotConstraints constraints;
    if (play_state.is_stop()) {
        constraints.mot.max_speed = 0.8;
    }

    return PlanRequest{start,
                       intent.motion_command,
                       constraints,
                       std::move(real_obstacles),
                       std::move(virtual_obstacles),
                       robot_trajectories_,
                       static_cast<unsigned int>(robot_id_),
                       world_state,
                       intent.priority,
                       &debug_draw_,
                       had_break_beam_};
}

Trajectory PlannerForRobot::plan_for_robot(const planning::PlanRequest& request) {
    // Try each planner in sequence until we find one that is applicable.
    // This gives the planners a sort of "priority" - this makes sense, because
    // the empty planner is always last.
    Trajectory trajectory;
    for (auto& planner : planners_) {
        // If this planner could possibly plan for this command, try to make
        // a plan.
        if (trajectory.empty() && planner->is_applicable(request.motion_command)) {
            current_planner_ = planner;
            trajectory = planner->plan(request);
        }

        // If it fails, or if the planner was not used, the trajectory will
        // still be empty. Reset the planner.
        if (trajectory.empty()) {
            current_planner_ = nullptr;
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
    } else {
        std::vector<rj_geometry::Point> path;
        std::transform(trajectory.instants().begin(), trajectory.instants().end(),
                       std::back_inserter(path),
                       [](const auto& instant) { return instant.position(); });
        debug_draw_.draw_path(path);
    }
    debug_draw_.draw_shapes(shared_state_->global_obstacles(), QColor(255, 0, 0, 30));
    debug_draw_.draw_shapes(request.virtual_obstacles, QColor(255, 0, 0, 30));

    debug_draw_.publish();

    return trajectory;
}

bool PlannerForRobot::robot_alive() const {
    return shared_state_->world_state()->our_robots.at(robot_id_).visible &&
           RJ::now() < shared_state_->world_state()->last_updated_time + RJ::Seconds(PARAM_timeout);
}

bool PlannerForRobot::is_done() const {
    // no segfaults
    if (current_planner_ == nullptr) {
        return false;
    }

    return current_planner_->is_done();
}

}  // namespace planning
