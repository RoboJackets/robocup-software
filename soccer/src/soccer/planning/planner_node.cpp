#include "planner_node.hpp"

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

PlannerNode::PlannerNode()
    : rclcpp::Node("planner", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      shared_state_(this),
      param_provider_{this, kPlanningParamModule} {
    robots_planners_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        auto planner =
            std::make_shared<PlannerForRobot>(i, this, &robot_trajectories_, &shared_state_);
        robots_planners_.emplace_back(std::move(planner));
    }
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
          fmt::format("planning_{}", robot_id_)} {
    planners_.push_back(std::make_shared<PathTargetPlanner>());
    planners_.push_back(std::make_shared<SettlePlanner>());
    planners_.push_back(std::make_shared<CollectPlanner>());
    planners_.push_back(std::make_shared<LineKickPlanner>());
    planners_.push_back(std::make_shared<PivotPathPlanner>());

    // The empty planner should always be last.
    planners_.push_back(std::make_shared<EscapeObstaclesPathPlanner>());

    // Set up ROS
    trajectory_pub_ = node_->create_publisher<Trajectory::Msg>(
        planning::topics::trajectory_pub(robot_id_), rclcpp::QoS(1).transient_local());
    // TODO: add macro for this to topic_names.hpp? (see above Trajectory)
    is_done_pub_ = node_->create_publisher<rj_msgs::msg::IsDone>(
        "planning/is_done/robot_" + std::to_string(robot_id_), rclcpp::QoS(1).transient_local());
    // 16ms = 60 hz (ish)
    timer_ =
        node_->create_wall_timer(16ms, std::bind(&PlannerForRobot::refresh_plan_request, this));

    intent_sub_ = node_->create_subscription<RobotIntent::Msg>(
        gameplay::topics::robot_intent_pub(robot_id_), rclcpp::QoS(1),
        // this is the callback executed when a RobotIntent is received
        [this](RobotIntent::Msg::SharedPtr intent) {  // NOLINT
            if (robot_alive()) {
                // save robot intent
                if (intent != nullptr) {
                    null_robot_intent = false;
                    latest_robot_intent_ = intent;
                }
            }
        });
    robot_status_sub_ = node_->create_subscription<rj_msgs::msg::RobotStatus>(
        radio::topics::robot_status_pub(robot_id_), rclcpp::QoS(1),
        [this](rj_msgs::msg::RobotStatus::SharedPtr status) {  // NOLINT
            had_break_beam_ = status->has_ball_sense;
        });
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
    rj_geometry::ShapeSet real_obstacles;
    // TODO(Kevin): this should be based on only if the current state is ball placement but I'm not
    // sure how to handle that right now the better long-term solution would be to make the ball
    // path not a global obstacle buta special one like def_area_obstacles see GameplayNode for
    // where all these obstacles are published from
    if (robot_id_ != 0 && robot_id_ != 1) {
        real_obstacles.add(global_obstacles);
    }
    rj_geometry::ShapeSet virtual_obstacles = intent.local_obstacles;
    if (!is_goalie) {
        virtual_obstacles.add(def_area_obstacles);
    }

    const auto robot_trajectories_hold = robot_trajectories_->get();
    std::array<const Trajectory*, kNumShells> planned_trajectories = {};

    for (int i = 0; i < kNumShells; i++) {
        // TODO(Kevin): check that priority works (seems like
        // robot_trajectories_ is passed on init, when no planning has occured
        // yet)
        const auto& [trajectory, priority] = robot_trajectories_hold.at(i);
        if (i != robot_id_ && priority >= intent.priority) {
            planned_trajectories.at(i) = trajectory.get();
        }
    }

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
                       planned_trajectories,
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

void PlannerForRobot::refresh_plan_request() {
    if (null_robot_intent) {
        return;
    }

    // create + pub a Trajectory from the last RobotIntent
    auto plan_request = make_request(rj_convert::convert_from_ros(*latest_robot_intent_));
    auto trajectory = plan_for_robot(plan_request);
    trajectory_pub_->publish(rj_convert::convert_to_ros(trajectory));

    // add that Trajectory to the shared_trajectories
    robot_trajectories_->put(robot_id_, std::make_shared<Trajectory>(std::move(trajectory)),
                             latest_robot_intent_->priority);

    // publish is_done status of robot
    rj_msgs::msg::IsDone msg;
    msg.is_done = this->is_done();
    is_done_pub_->publish(msg);
}

}  // namespace planning
