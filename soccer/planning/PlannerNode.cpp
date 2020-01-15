#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/PivotPathPlanner.hpp"
#include "planning/planner/CapturePlanner.hpp"
#include "planning/planner/EscapeObstaclesPathPlanner.hpp"
#include "PlannerNode.hpp"
#include "Robot.hpp"

namespace Planning {

PlannerNode::PlannerNode(Context* context) : context_(context), plannerIdx(Num_Shells, -1) {
    planners_.push_back(std::make_unique<PathTargetPlanner>());
    planners_.push_back(std::make_unique<CapturePlanner>());
    planners_.push_back(std::make_unique<PivotPathPlanner>());

    // The empty planner should always be last.
    planners_.push_back(std::make_unique<EscapeObstaclesPathPlanner>());
}

void PlannerNode::run() {
    Geometry2d::ShapeSet globalObstacles = context_->globalObstacles;
    Geometry2d::ShapeSet globalObstaclesWithGoalZones = globalObstacles;
    globalObstaclesWithGoalZones.add(context_->goalZoneObstacles);

    std::vector<PlanRequest> requests;
    for (OurRobot* robot : context_->state.self) {
        if (!robot) {
            continue;
        }

        if (!robot->visible() ||
            context_->game_state.state == GameState::Halt) {
            Trajectory inactivePath{{}};
            inactivePath.setDebugText("INACTIVE");
            robot->setPath(std::move(inactivePath));
            continue;
        }

        // Visualize local obstacles
        for (auto &shape : robot->localObstacles().shapes()) {
            context_->debug_drawer.drawShape(shape, Qt::black,
                                             "LocalObstacles");
        }

        bool robotIgnoreGoalZone = robot->shell() == context_->goalie_id ||
                                   robot->isPenaltyKicker ||
                                   robot->isBallPlacer;
        auto &globalObstaclesForBot =
                robotIgnoreGoalZone
                ? globalObstacles
                : globalObstaclesWithGoalZones;

        // create and visualize obstacles
        Geometry2d::ShapeSet staticObstacles =
                robot->collectStaticObstacles(
                        globalObstaclesForBot,
                        !robotIgnoreGoalZone);

        // Construct a plan request.
        if (robot->motionCommand()) {
            const RobotState &robotState = robot->state();
            //todo(Ethan) delete this
            assert(robotState.timestamp <= RJ::now());
            requests.emplace_back(
                    context_, RobotInstant{robotState.pose, robotState.velocity,
                                           robotState.timestamp},
                    *robot->motionCommand(),
                    robot->robotConstraints(), robot->path_movable(),
                    std::move(staticObstacles), std::vector<DynamicObstacle>{},
                    robot->shell(), robot->getPlanningPriority());
        }
    }
    std::sort(requests.begin(), requests.end(), [](const PlanRequest& pr1, const PlanRequest& pr2) {
        return pr1.priority > pr2.priority;
    });
    std::vector<DynamicObstacle> dynamicObstacles;
    for(PlanRequest& request : requests) {
        //update robot specific obstacles
        request.dynamic_obstacles = dynamicObstacles;

        //complete the plan request
        OurRobot* robot = context_->state.self[request.shellID];
        Trajectory plannedPath = PlanForRobot(std::move(request));
        robot->setPath(std::move(plannedPath));
//        dynamicObstacles.emplace_back(std::make_shared<Geometry2d::Circle>(robot->pos(), Robot_Radius), robot->path()); todo(Ethan) uncomment this

        //draw debug info
        robot->path().draw(&context_->debug_drawer, robot->pos() + Geometry2d::Point(.1,0));
        context_->debug_drawer.drawText((const char*[]) {
                "EmptyCommand",
                "PathTargetCommand",
                "WorldVelTargetCommand",
                "PivotCommand",
                "TuningPathCommand",
                "CaptureCommand",
                "InterceptCommand",
                "SettleCommand",
                "CollectCommand"
        }[robot->motionCommand()->index()], robot->pos()+Geometry2d::Point(.1,.3), QColor(100, 100, 255, 100));
        context_->debug_drawer.drawText(QString("Path Age: ") + std::to_string(RJ::Seconds(RJ::now() - robot->path().begin_time()).count()).c_str(), robot->pos()+Geometry2d::Point(.1, -.2), QColor(100, 100, 255, 100));
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
    for (int i = 0; i < planners_.size(); i++) {
        if (planners_[i]->isApplicable(request.motionCommand)) {
            //clear path when planner changes
            if(plannerIdx[request.shellID] != i) {
                plannerIdx[request.shellID] = i;
                request.prevTrajectory = Trajectory{{}};
            }
            RobotInstant startInstant = request.start;
            RJ::Time t0 = RJ::now();
            Trajectory path = planners_[i]->plan(std::move(request));
            if(path.empty()) {
                path = Trajectory{{startInstant}};
                debugLog("Empty Path. Planner: " + planners_[i]->name());
            }
            return std::move(path);
        }
    }
    std::cerr << "No valid planner! Did you forget to specify a default planner?"
              << std::endl;
    Trajectory result{{}};
    result.setDebugText("Error: No Valid Planners");
    return std::move(result);
}

} // namespace Planning
