#include "CapturePlanner.hpp"
#include <Configuration.hpp>
#include "CompositePath.hpp"
#include "MotionInstant.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool CapturePlanner::shouldReplan(const PlanRequest& planRequest) const {
    return true;
}

std::unique_ptr<Path> CapturePlanner::run(PlanRequest& planRequest) {
    const InterceptCommand& command = 
    dynamic_cast<const InterceptCommand&>(*planRequest.motionCommand);

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    targetFinalCaptureDirectionPos = command.target;

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

    // How much of the ball speed to use to dampen the bounce
    const float ballSpeedPercentForDampen = 0.1; // %

    // Ball speed cutoff to decide when to catch it from behind or get in front of it
    const float minSpeedToIntercept = 0.1; // m/s

    // Max angle from the ball vector when trying to bounce the ball
    // on dampen when trying to speed up actions after captures
    const float maxAngleoffBallForDampen = 45; // deg

    // Search settings when trying to find the correct intersection location
    // between the fast moving ball and our collecting robot
    const RJ::Seconds searchStartTime = RJ::Seconds(0.1);
    const RJ::Seconds searchEndTime = RJ::Seconds(6);
    const RJ::Seconds searchIncTime = RJ::Seconds(0.2);

    // TODO: Do more than try to intercept a moving ball

    // Try find best point to intercept using old method
    // where we check ever X seconds along the ball velocity line
    // TODO: Try the pronav algorithm
    for (RJ::Seconds t = searchStarttime; t < searchEndTime; t += searchIncTime) {
        MotionInstant ballNow = ball.predict(curTime + t);
        MotionInstant targetRobotIntersection(ballNow.pos);

        // TODO: Take the targetFinalCaptureDirection into account
        // Use the mouth to center vector, rotate by X degrees
        // Take the delta between old and new mouth vector and move
        // targetRobotIntersection by that amount

        // TODO: Improve velocity target
        //       May be able to just dampen here instead of another state
        targetRobotIntersection.vel = Point(0, 0);

        std::unique_ptr<Path> path =
            RRTPlanner:generatePath(points, obstacles, motionConstraints, 
                                    startInstant.vel, targetRobotIntersection.vel);

        RJ::Seconds timeOfArrival = path->getDuration();

        if (path) {
            if (timeOfArrival <= t) {
                path->setDebugText(QString::number(timeOfArrival) + " : " 
                                   QString::number(t));

                return make_unique<AngleFunctionPath>(
                    std::move(path), angleFunctionForCommandType(
                                    FacePointCommand(Command.target)));
            }
        }
    }
}
}