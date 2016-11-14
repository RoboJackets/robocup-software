#include "TrapezoidalPath.hpp"

using namespace Geometry2d;
namespace Planning {

TrapezoidalPath::TrapezoidalPath(Geometry2d::Point startPos, float startSpeed,
                                 Geometry2d::Point endPos, float endSpeed,
                                 const MotionConstraints& constraints)
    : _startPos(startPos),
      _startSpeed(std::min(startSpeed, constraints.maxSpeed)),
      _endPos(endPos),
      _endSpeed(endSpeed),
      _pathLength((startPos - endPos).mag()),
      _maxAcc(constraints.maxAcceleration),
      _maxSpeed(constraints.maxSpeed),
      _pathDirection((endPos - startPos).normalized()),
      // Precalculate the duration of the path
      _duration(Trapezoidal::getTime(_pathLength,  // distance
                                     _pathLength,  // pathLength
                                     _maxSpeed, _maxAcc, _startSpeed,
                                     _endSpeed)) {}

boost::optional<RobotInstant> TrapezoidalPath::evaluate(
    RJ::Seconds time) const {
    float distance;
    float speedOut;
    bool valid = TrapezoidalMotion(_pathLength,   // PathLength
                                   _maxSpeed,     // maxSpeed
                                   _maxAcc,       // maxAcc
                                   time.count(),  // time
                                   _startSpeed,   // startSpeed
                                   _endSpeed,     // endSpeed
                                   distance,      // posOut
                                   speedOut);     // speedOut
    if (!valid) return boost::none;

    return RobotInstant(MotionInstant(_pathDirection * distance + _startPos,
                                      _pathDirection * speedOut));
}

bool TrapezoidalPath::hit(const Geometry2d::ShapeSet& obstacles,
                          RJ::Seconds initialTime, RJ::Seconds* hitTime) const {
    std::set<std::shared_ptr<Shape>> startHitSet = obstacles.hitSet(_startPos);
    for (RJ::Seconds t = initialTime; t < _duration; t += RJ::Seconds(0.1)) {
        auto instant = evaluate(t);
        if (instant) {
            for (auto& shape : obstacles.shapes()) {
                // If the shape is in the original hitSet, it is ignored
                if (startHitSet.find(shape) != startHitSet.end()) {
                    continue;
                }

                if (shape->hit(instant->motion.pos)) {
                    if (hitTime) {
                        *hitTime = t;
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

std::unique_ptr<Path> TrapezoidalPath::subPath(RJ::Seconds startTime,
                                               RJ::Seconds endTime) const {
    debugThrow("This function is not implemented");
    return nullptr;
}

}  // namespace Planning
