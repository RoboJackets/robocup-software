#include "PlannerUtil.hpp"
#include "Geometry2d/Point.hpp"
#include "Geometry2d/Segment.hpp"
#include "Geometry2d/Rect.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/MotionConstraints.hpp"
#include "planning/trajectory/RRTUtil.hpp"

namespace Planning {
using Geometry2d::Point;

Point projectPointIntoField(Point targetPoint, const Geometry2d::Rect& fieldRect, Point ballPoint){
    auto intersectReturn = fieldRect.intersects(Geometry2d::Segment(ballPoint, targetPoint));

    bool validIntersect = std::get<0>(intersectReturn);
    std::vector<Point> intersectPts = std::get<1>(intersectReturn);

    // If the ball intersects the field at some point
    // Just get the intersect point as the new target
    if (validIntersect) {
        // Sorts based on distance to intercept target
        // The closest one is the intercept point which the ball moves
        // through leaving the field Not the one on the other side of the
        // field
        // Choose a point just inside the field
        targetPoint = *std::min_element(intersectPts.begin(), intersectPts.end(), [&](Point a, Point b) {
            return (a - targetPoint).mag() <
                   (b - targetPoint).mag();
        });

        // Doesn't intersect
        // project the ball into the field
    } else {
        // Simple projection
        targetPoint.x() = std::clamp(targetPoint.x(),
                                     (double)fieldRect.minx(), (double)fieldRect.maxx());
        targetPoint.y() = std::clamp(targetPoint.y(),
                                     (double)fieldRect.miny(), (double)fieldRect.maxy());
    }
    return targetPoint;
}

Point bruteForceGoal(const PlanRequest& request, Geometry2d::Point avgBallVel,
        double bufferTime,
        double searchStart, double searchEnd, double searchInc) {
    double distance = searchStart;
    const Ball& ball = request.context->state.ball;
    Point targetPoint;
    double targetAngle = fixAngleRadians(avgBallVel.angle() + M_PI);
    const Geometry2d::Rect &fieldRect = Field_Dimensions::Current_Dimensions.FieldRect();
    Point startPoint = request.start.pose.position();
    RJ::Time curTime = RJ::now();
    Trajectory path{{}};
    int its = 0;
    while (distance < searchEnd) {
        Point futureBallPoint =
                ball.pos + avgBallVel.normalized(distance);
        RJ::Time futureBallTime = ball.estimateTimeTo(futureBallPoint, &targetPoint);
        //todo(Ethan) handle futureBallTime == RJ::Time::max() ?
        RJ::Seconds ballPathDuration = futureBallTime - curTime;
        if(!fieldRect.containsPoint(targetPoint)) {
            targetPoint = projectPointIntoField(targetPoint, fieldRect, ball.pos);
            break;
        }
        RobotInstant pathTarget{Geometry2d::Pose{targetPoint, targetAngle},
                                {}, RJ::now()};
        path = RRTTrajectory(request.start, pathTarget, request.constraints.mot, request.static_obstacles);
        if(!path.empty() && path.duration() * bufferTime <= ballPathDuration) {
            break;
        }
        distance += searchInc;
        its++;
    }
    printf("brute force took %.3f sec, its: %d\n", RJ::Seconds(RJ::now()-curTime).count(), its);
    return targetPoint;
}
}