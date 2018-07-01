#include "InterceptPlanner.hpp"

#include <Configuration.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include <Constants.hpp>

#include "RRTPlanner.hpp"
#include "MotionInstant.hpp"


namespace Planning{

bool InterceptPlanner::shouldReplan(const PlanRequest& planRequest) const {
    // Intercept point is > 1 radius from current ball path
    std::cout << "Should Replace" << std::endl;
    return false;
}

std::unique_ptr<Path> InterceptPlanner::run(PlanRequest& planRequest) {
    const InterceptCommand& command = 
        dynamic_cast<const InterceptCommand&>(*planRequest.motionCommand);

    targetInterceptPos = command.target;

    // Start state for the specified robot
    const MotionInstant& startInstant = planRequest.start;
    // All the max velocity / acceleration constraints for translation / rotation
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const RobotConstraints& robotConstraints = planRequest.constraints;
    // List of obstacles
    Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::vector<DynamicObstacle>& dynamicObstacles = planRequest.dynamicObstacles;

    SystemState& systemState = planRequest.systemState;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();

    // Intercept the ball at closest point to the ball vel
    
    // Get closest point to ball vel vector
    // Calc ball time to pos
    // Find path to point
    // Up end vel until time to pos = ball pos

    MotionInstant ballCurrent = ball.predict(curTime);
    

    // Time for ball to hit target point
    Geometry2d::Point targetPosOnLine;
    RJ::Seconds ballToPointTime = ball.estimateTimeTo(targetInterceptPos, &targetPosOnLine) - curTime;
    // Time for robot to hit target point
    RJ::Seconds botToPointTime(0);

    // vector from robot to target
    Geometry2d::Point botToTarget = (targetPosOnLine - startInstant.pos);
    // Normalized vector from robot to target
    Geometry2d::Point botToTargetNorm = botToTarget.normalized();

    std::unique_ptr<Path> path;
    std::vector<Geometry2d::Point> startEndPoints{startInstant.pos, targetPosOnLine};

    // Scale the end velocity by % of max velocity to see if we can reach the target
    // at the same time as the ball
    for (double mag = 0.0; mag <= 1.0; mag += .05) {
        path = RRTPlanner::generatePath(startEndPoints, obstacles, motionConstraints,
                                        startInstant.vel, 
                                        mag*motionConstraints.maxSpeed * botToTargetNorm);

        // First path where we can reach the point at or before the ball
        // If the end velocity is not 0, you should reach the point as close
        // to the ball time as possible to just ram it
        if (path) {
            botToPointTime = path->getDuration();

            if (botToPointTime <= ballToPointTime) {
                path->setDebugText("Found Path. RT " + QString::number(botToPointTime.count(), 'g', 2) +
                                   " BT " + QString::number(ballToPointTime.count(), 'g', 2) +
                                   " FS " + QString::number(mag));
                
                return std::make_unique<AngleFunctionPath>(
                    std::move(path), 
                    angleFunctionForCommandType(FacePointCommand(ball.pos))); 
            }
        }
    }

    QString debug;

    MotionInstant target;
    target.pos = targetPosOnLine;
    
    // If we are already almost at the correct point, dont speed out of the way
    // We may not find a path that is shorter to the ball because our ball filter is off a lot...
    if (botToTarget.mag() < Robot_Radius / 2) {
        target.vel = Geometry2d::Point(0,0);
        debug = "AtPoint";
     } else {
        target.vel = motionConstraints.maxSpeed * botToTargetNorm;
        debug = "GivingUp";
     }

    // Couldn't find a good path, give up
    std::unique_ptr<MotionCommand> rrtCommand =
        std::make_unique<PathTargetCommand>(target);

    PlanRequest request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                      robotConstraints, nullptr, obstacles,
                                      dynamicObstacles, planRequest.shellID);
    path = rrtPlanner.run(request);
    path->setDebugText(debug + ". RT " + QString::number(botToPointTime.count(), 'g', 2) +
                       " BT " + QString::number(ballToPointTime.count(), 'g', 2));

    return std::make_unique<AngleFunctionPath>(
        std::move(path),
        angleFunctionForCommandType(FacePointCommand(ball.pos)));
}
}