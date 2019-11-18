#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/DirectTargetPathPlanner.hpp"
#include "planning/planner/SettlePathPlanner.hpp"
#include "planning/planner/PivotPathPlanner.hpp"
#include "PlannerNode.hpp"
#include "Robot.hpp"

namespace Planning {

PlannerNode::PlannerNode(Context* context) : context_(context) {
    planners_.push_back(std::make_unique<PathTargetPlanner>());
    planners_.push_back(std::make_unique<SettlePathPlanner>());
//    planners_.push_back(std::make_unique<PivotPathPlanner>());

    // The empty planner should always be last.
    planners_.push_back(std::make_unique<EmptyPlanner>());
}

void PlannerNode::run() {
    // TODO(Kyle): Get obstacles.
    /*
    Geometry2d::ShapeSet globalObstacles =
            _gameplayModule->globalObstacles();
    Geometry2d::ShapeSet globalObstaclesWithGoalZones = globalObstacles;
    Geometry2d::ShapeSet goalZoneObstacles =
            _gameplayModule->goalZoneObstacles();
        */
    Geometry2d::ShapeSet globalObstacles;
    Geometry2d::ShapeSet globalObstaclesWithGoalZones = globalObstacles;
    Geometry2d::ShapeSet goalZoneObstacles;
    globalObstaclesWithGoalZones.add(goalZoneObstacles);

    for (OurRobot* robot : context_->state.self) {
        if (!robot) {
            continue;
        }

        if (!robot->visible() || context_->game_state.state == GameState::Halt) {
            robot->setPath(Trajectory({}));
            continue;
        }

        // Visualize local obstacles
        for (auto& shape : robot->localObstacles().shapes()) {
            context_->debug_drawer.drawShape(shape, Qt::black,
                                            "LocalObstacles");
        }

        // TODO(Kyle) Get the goalie ID
        int goalieID = 0;

        bool robotIgnoreGoalZone = robot->shell() == goalieID || robot->isPenaltyKicker || robot->isBallPlacer;
        auto& globalObstaclesForBot =
                robotIgnoreGoalZone
                ? globalObstacles
                : globalObstaclesWithGoalZones;

        // create and visualize obstacles
        Geometry2d::ShapeSet staticObstacles =
                robot->collectStaticObstacles(
                        globalObstaclesForBot,
                        !robotIgnoreGoalZone);

        // TODO(Kyle): Collect dynamic obstacles
        /*
        std::vector<Planning::DynamicObstacle> dynamicObstacles =
                robot->collectDynamicObstacles();
                */

        // Construct a plan request.
        PlanRequest request = PlanRequest(
                context_, robot->state(), robot->motionCommand(),
                robot->robotConstraints(), robot->path_movable(),
                staticObstacles, robot->shell(), robot->getPlanningPriority());

        robot->setPath(std::move(PlanForRobot(std::move(request))));
    }

    // Visualize obstacles
    for (auto& shape : globalObstacles.shapes()) {
        context_->debug_drawer.drawShape(shape, Qt::black,
                                        "Global Obstacles");
    }
}

Trajectory PlannerNode::PlanForRobot(Planning::PlanRequest&& request) {
    // Try each planner in sequence until we find one that is applicable.
    // This gives the planners a sort of "priority" - this makes sense, because
    // the empty planner is always last.
    for (auto& planner : planners_) {
        if (planner->isApplicable(request.motionCommand)) {
//            std::cout << "Planner: " << planner->name() << std::endl;
            return std::move(planner->plan(std::move(request)));
        } else {
//            std::cout << "Planner " << planner->name() << " is not applicable!" << std::endl; todo(Ethan) uncomment this
        }
    }

    std::cerr << "No valid planner! Did you forget to specify a default planner?"
              << std::endl;
    return Trajectory({});
}

} // namespace Planning
