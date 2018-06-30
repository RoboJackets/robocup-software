#include "InterceptPlanner.hpp"

#include <Configuration.hpp>
#include <motion/TrapezoidalMotion.hpp>

#include <iostream>

#include "RRTPlanner.hpp"
#include "MotionInstant.hpp"

namespace Planning{

bool InterceptPlanner::shouldReplan(const PlanRequest& planRequest) const {
    // Intercept point is > 1 radius from current ball path
    std::cout << "Should Replace" << std::endl;
    return false;
}

std::unique_ptr<Path> InterceptPlanner::run(PlanRequest& planRequest) {
    std::cout << "Entering run" << std::endl;

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

    SystemState systemState = planRequest.systemState;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();

    // Intercept the ball at closest point to the ball vel
    
    // Get closest point to ball vel vector
    // Calc ball time to pos
    // Find path to point
    // Up end vel until time to pos = ball pos

    MotionInstant ballCurrent = ball.predict(curTime);
    
    // Normalized vector between target and robot
    Geometry2d::Point botToTargetNorm = (targetInterceptPos - startInstant.pos).normalized();

    // Time for ball to hit target point
    RJ::Seconds ballToPointTime = ball.estimateTimeTo(targetInterceptPos) - curTime;

    std::unique_ptr<Path> path;
    std::vector<Geometry2d::Point> startEndPoints{startInstant.pos, targetInterceptPos};

    std::cout << "Starting to check velocities" << std::endl;

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
            if (path->getDuration() <= ballToPointTime) {
                std::cout << "Returning" << std::endl;
                path->setDebugText("Found Path " + QString::number(path->getDuration().count()));
                std::cout << "Done Printing" << std::endl;
                
                return std::make_unique<AngleFunctionPath>(
                    std::move(path), 
                    angleFunctionForCommandType(FacePointCommand(ball.pos))); 
            }
        }
    }

    std::cout << "Giving up" << std::endl;

    MotionInstant target(targetInterceptPos, Geometry2d::Point(0, 0));

    // Couldn't find a good path, give up
    std::unique_ptr<MotionCommand> rrtCommand =
        std::make_unique<PathTargetCommand>(target);

    PlanRequest request = PlanRequest(systemState, startInstant, std::move(rrtCommand),
                                      robotConstraints, nullptr, obstacles,
                                      dynamicObstacles, planRequest.shellID);
    path = rrtPlanner.run(request);
    path->setDebugText("Giving Up");

    std::cout << "Returning" << std::endl;

    return std::make_unique<AngleFunctionPath>(
        std::move(path),
        angleFunctionForCommandType(FacePointCommand(targetInterceptPos)));
}
}