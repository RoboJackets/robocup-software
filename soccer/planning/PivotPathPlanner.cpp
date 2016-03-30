#include "PivotPathPlanner.hpp"
#include "TrapezoidalPath.hpp"
#include "EscapeObstaclesPathPlanner.hpp"
#include <Configuration.hpp>
#include <cmath>
#include <boost/range/irange.hpp>
#include "RRTPlanner.hpp"

using namespace std;
using namespace Geometry2d;

namespace Planning {

bool PivotPathPlanner::shouldReplan(MotionInstant startInstant,
                                    const PivotCommand command,
                                    const MotionConstraints& motionConstraints,
                                    const Geometry2d::ShapeSet* obstacles,
                                    const Path* prevPath) {

    if (!prevPath) {
        return true;
    } else {
        //TODO ashaw37: make this better
        float radius = command.radius;
        auto pivotPoint = command.pivotPoint;
        auto pivotTarget = command.pivotTarget;
        auto endTarget = pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
        float targetChange = (prevPath->end().motion.pos - endTarget).mag();

        if (targetChange > SingleRobotPathPlanner::goalChangeThreshold()) {
            //return true;
        }
    }
    return false;
}

std::unique_ptr<Path> PivotPathPlanner::run(
    MotionInstant startInstant, const MotionCommand* cmd,
    const MotionConstraints& motionConstraints,
    const Geometry2d::ShapeSet* obstacles, std::unique_ptr<Path> prevPath) {

    PivotCommand command = *dynamic_cast<const PivotCommand*>(cmd);

    if (shouldReplan(startInstant, command, motionConstraints, obstacles, prevPath.get())) {

        float radius = command.radius;
        auto pivotPoint = command.pivotPoint;
        auto pivotTarget = command.pivotTarget;
        auto endTarget = pivotPoint + (pivotPoint - pivotTarget).normalized(radius);
        vector<Point> points;

        float startAngle = pivotPoint.angleTo(startInstant.pos);
        float targetAngle = pivotPoint.angleTo(endTarget);
        float change = fixAngleRadians(targetAngle - startAngle);

        const int interpolations = 10;

        points.push_back(startInstant.pos);
        for(int i=1; i<=interpolations; i++) {
            float percent = (float) i / interpolations;
            float angle = startAngle + change * percent;
            Point point = Point::direction(angle).normalized(radius) + pivotPoint;
            points.push_back(point);
        }
        unique_ptr<Path> path = RRTPlanner::generatePath(points, *obstacles, motionConstraints, startInstant.vel, Point(0,0));
        std::function<AngleInstant(MotionInstant)> function =
                [pivotPoint](MotionInstant instant) {
                    return AngleInstant(instant.pos.angleTo(pivotPoint));
                };
        return make_unique<AngleFunctionPath>(move(path), function);;
    } else {
        return std::move(prevPath);
    }
}
}
