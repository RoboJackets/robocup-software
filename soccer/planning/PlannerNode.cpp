#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/DirectTargetPathPlanner.hpp"
#include "planning/planner/SettlePathPlanner.hpp"
#include "planning/planner/PivotPathPlanner.hpp"
#include "planning/planner/CollectPathPlanner.hpp"
#include "planning/planner/EscapeObstaclesPathPlanner.hpp"
#include "PlannerNode.hpp"
#include "Robot.hpp"

namespace Planning {

PlannerNode::PlannerNode(Context* context) : context_(context) {
    planners_.push_back(std::make_unique<PathTargetPlanner>());
    planners_.push_back(std::make_unique<SettlePathPlanner>());
    planners_.push_back(std::make_unique<CollectPathPlanner>());
    planners_.push_back(std::make_unique<PivotPathPlanner>());

    // The empty planner should always be last.
    planners_.push_back(std::make_unique<EscapeObstaclesPathPlanner>());
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
            Trajectory inactivePath{{}};
            inactivePath.setDebugText("INACTIVE");
            robot->setPath(std::move(inactivePath));
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
        if(robot->motionCommand()) {
            const RobotState& robotState = robot->state();
            PlanRequest request = PlanRequest(
                    context_, RobotInstant{robotState.pose, robotState.velocity, robotState.timestamp}, *robot->motionCommand(),
                    robot->robotConstraints(), robot->path_movable(),
                    staticObstacles, robot->shell(), robot->getPlanningPriority());

            //complete the plan request
            Trajectory plannedPath = PlanForRobot(std::move(request));
            plannedPath.draw(&context_->debug_drawer, robot->pos() + Geometry2d::Point(.1,0));
            robot->setPath(std::move(plannedPath));
            context_->debug_drawer.drawText((const char*[]) {
                "Empty",
                "RRT",
                "WorldVel",
                "Pivot",
                "Direct",
                "TuningPath",
                "Settle",
                "Collect",
                "LineKick",
                "Intercept"
            }[robot->motionCommand()->index()], robot->pos()+Geometry2d::Point(.1,.3), QColor(100, 100, 255));
            context_->debug_drawer.drawText(QString("Path Age: ") + std::to_string(RJ::Seconds(RJ::now() - robot->path().begin_time()).count()).c_str(), robot->pos()+Geometry2d::Point(.1, -.2), Qt::white);
        }
    }

    // Visualize obstacles
    for (auto& shape : globalObstacles.shapes()) {
        context_->debug_drawer.drawShape(shape, Qt::black,
                                        "Global Obstacles");
    }
}

Trajectory PlannerNode::PlanForRobot(Planning::PlanRequest&& request) {
    Geometry2d::Point robotPos =  request.context->state.self[request.shellID]->pos();
    // Try each planner in sequence until we find one that is applicable.
    // This gives the planners a sort of "priority" - this makes sense, because
    // the empty planner is always last.
    DebugDrawer* drawer = &request.context->debug_drawer;
    for (auto& planner : planners_) {
        if (planner->isApplicable(request.motionCommand)) {
            return planner->plan(std::move(request));
        } else {
//            std::cout << "Planner " << planner->name() << " is not applicable!" << std::endl; todo(Ethan) uncomment this
        }
    }
    std::cerr << "No valid planner! Did you forget to specify a default planner?"
              << std::endl;
    Trajectory result{{}};
    result.setDebugText("Error: No Valid Planners");
    return std::move(result);
}

} // namespace Planning
