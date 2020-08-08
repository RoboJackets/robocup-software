#include "planner_node.hpp"

#include <rj_constants/topic_names.hpp>

#include "instant.hpp"
#include "robot.hpp"
#include "planning/planner/collect_planner.hpp"
#include "planning/planner/escape_obstacles_path_planner.hpp"
#include "planning/planner/line_kick_planner.hpp"
#include "planning/planner/path_target_planner.hpp"
#include "planning/planner/pivot_path_planner.hpp"
#include "planning/planner/settle_planner.hpp"

namespace Planning {

PlannerNode::PlannerNode(Context* context) : context_(context) {
    robots_planners_.resize(kNumShells);

    world_state_queue_ = std::make_unique<AsyncWorldStateMsgQueue>(
        "planner_node_game_state_sub", vision_filter::topics::kWorldStatePub);
}

using namespace Geometry2d;
void PlannerNode::run() {
    const WorldStateMsg::SharedPtr world_state_msg = world_state_queue_->get();
    if (world_state_msg == nullptr) {
        return;
    }

    const WorldState world_state = rj_convert::convert_from_ros(*world_state_msg);
    const ShapeSet& global_obstacles = context_->global_obstacles;
    const ShapeSet& goal_zones = context_->goal_zone_obstacles;
    const auto& robot_intents = context_->robot_intents;
    DebugDrawer* debug_drawer = &context_->debug_drawer;
    auto* trajectories = &context_->trajectories;
    int goalie_id = context_->our_info.goalie;

    if (context_->game_state.state == GameState::Halt) {
        context_->trajectories.fill(Trajectory());
        return;
    }

    std::array<Trajectory*, kNumShells> planned{};

    // Sort the robots by priority
    std::array<int, kNumShells> shells{};
    for (int i = 0; i < kNumShells; i++) {
        shells[i] = i;
    }
    std::sort(shells.begin(), shells.end(), [&](int a, int b) {
        return robot_intents.at(a).priority > robot_intents.at(b).priority;
    });

    for (int i = 0; i < kNumShells; i++) {
        unsigned int shell = shells.at(i);
        const auto& robot = world_state.our_robots.at(shell);
        const auto& intent = robot_intents.at(shell);

        if (!robot.visible) {
            // For invalid robots, early return with an empty path.
            trajectories->at(shell) = Trajectory();
            planned.at(shell) = &trajectories->at(shell);
            continue;
        }

        RobotInstant start{robot.pose, robot.velocity, robot.timestamp};

        ShapeSet local_obstacles = intent.local_obstacles;
        if (goalie_id != shell) {
            local_obstacles.add(goal_zones);
        }

        if (debug_drawer != nullptr) {
            for (const auto& shape : local_obstacles.shapes()) {
                debug_drawer->draw_shape(shape, QColor(0, 0, 0, 16));
            }
        }

        // TODO(#1500): Put motion constraints in intent.
        PlanRequest request{start,
                            intent.motion_command,
                            RobotConstraints(),
                            global_obstacles,
                            local_obstacles,
                            planned,
                            shell,
                            &world_state,
                            intent.priority,
                            debug_drawer};

        Trajectory trajectory = robots_planners_.at(shell).plan_for_robot(request);
        trajectory.draw(&context_->debug_drawer);
        trajectories->at(shell) = std::move(trajectory);

        planned.at(shell) = &trajectories->at(shell);
    }
}

PlannerForRobot::PlannerForRobot() {
    planners_.push_back(std::make_unique<PathTargetPlanner>());
    planners_.push_back(std::make_unique<SettlePlanner>());
    planners_.push_back(std::make_unique<CollectPlanner>());
    planners_.push_back(std::make_unique<LineKickPlanner>());
    planners_.push_back(std::make_unique<PivotPathPlanner>());

    // The empty planner should always be last.
    planners_.push_back(std::make_unique<EscapeObstaclesPathPlanner>());
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
        std::cerr << "No valid planner! Did you forget to specify a default planner?" << std::endl;
        trajectory = Trajectory{{request.start}};
        trajectory.set_debug_text("Error: No Valid Planners");
    }

    return trajectory;
}

}  // namespace Planning
