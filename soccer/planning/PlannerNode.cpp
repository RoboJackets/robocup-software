#include <Robot.hpp>
#include <planning/PlannerNode.hpp>
#include <planning/planners/IndependentMultiRobotPathPlanner.hpp>

namespace Planning {
PlannerNode::PlannerNode(Context* context)
    : context_{context},
      path_planner_{
          std::make_unique<Planning::IndependentMultiRobotPathPlanner>()} {}

using namespace Geometry2d;
void PlannerNode::run() {
    /// Collect global obstacles
    const GlobalObstacles global_obstacles = getGlobalObstacles();

    // Build a plan request for each robot.
    std::map<int, Planning::PlanRequest> requests =
        buildPlanRequests(global_obstacles);

    // Run path planner and set the path for each robot that was planned for
    auto pathsById = path_planner_->run(std::move(requests));
    for (auto& entry : pathsById) {
        OurRobot* r = context_->state.self[entry.first];
        auto& path = entry.second;
        path->draw(&context_->debug_drawer, Qt::magenta, "Planning");
        path->drawDebugText(&context_->debug_drawer);
        r->setPath(std::move(path));

        r->angleFunctionPath().angleFunction =
            angleFunctionForCommandType(r->rotationCommand());
    }

    // Visualize obstacles
    for (const auto& shape : global_obstacles.global_obstacles.shapes()) {
        context_->debug_drawer.drawShape(shape, Qt::black, "Global Obstacles");
    }
}

GlobalObstacles PlannerNode::getGlobalObstacles() {
    const Geometry2d::ShapeSet global_obstacles = context_->globalObstacles;
    const Geometry2d::ShapeSet goal_zone_obstacles =
        context_->goalZoneObstacles;
    Geometry2d::ShapeSet global_obstacles_with_goal_zones = global_obstacles;
    global_obstacles_with_goal_zones.add(goal_zone_obstacles);

    return {global_obstacles, goal_zone_obstacles,
            global_obstacles_with_goal_zones};
}

std::map<int, Planning::PlanRequest> PlannerNode::buildPlanRequests(
    const GlobalObstacles& global_obstacles) {
    std::map<int, Planning::PlanRequest> requests{};
    for (OurRobot* r : context_->state.self) {
        if (r != nullptr && r->visible()) {
            if (context_->game_state.state == GameState::Halt) {
                r->setPath(nullptr);
                continue;
            }

            // Visualize local obstacles
            for (const auto& shape : r->localObstacles().shapes()) {
                context_->debug_drawer.drawShape(shape, Qt::black,
                                                 "LocalObstacles");
            }

            const auto& globalObstaclesForBot =
                (r->shell() == context_->game_state.getGoalieId() ||
                 r->isPenaltyKicker || r->isBallPlacer)
                    ? global_obstacles.global_obstacles
                    : global_obstacles.global_obstacles_with_goal_zones;

            // create and visualize obstacles
            Geometry2d::ShapeSet staticObstacles = r->collectStaticObstacles(
                globalObstaclesForBot,
                !(r->shell() == context_->game_state.getGoalieId() ||
                  r->isPenaltyKicker || r->isBallPlacer));

            std::vector<Planning::DynamicObstacle> dynamicObstacles =
                r->collectDynamicObstacles();

            requests.emplace(
                r->shell(),
                Planning::PlanRequest(
                    context_, Planning::MotionInstant(r->pos(), r->vel()),
                    r->motionCommand()->clone(), r->robotConstraints(),
                    std::move(r->angleFunctionPath().path),
                    std::move(staticObstacles), std::move(dynamicObstacles),
                    r->shell(), r->getPlanningPriority()));
        }
    }
    return requests;
}
}  // namespace Planning
