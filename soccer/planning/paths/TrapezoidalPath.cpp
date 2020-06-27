#include "TrapezoidalPath.hpp"

#include <stdexcept>

using namespace Geometry2d;
namespace Planning {

TrapezoidalPath::TrapezoidalPath(Geometry2d::Point startPos, double startSpeed,
                                 Geometry2d::Point endPos, double endSpeed,
                                 const MotionConstraints constraints)
    : _startPos(startPos),
      _startSpeed(std::min(startSpeed, constraints.maxSpeed)),
      _endPos(endPos),
      _endSpeed(endSpeed),
      _pathLength((startPos - endPos).mag()),
      _maxAcc(constraints.maxAcceleration),
      _maxSpeed(constraints.maxSpeed),
      _constraints(constraints),
      _pathDirection((endPos - startPos).normalized()),
      // Precalculate the duration of the path
      _duration(Trapezoidal::getTime(_pathLength,  // distance
                                     _pathLength,  // pathLength
                                     _maxSpeed, _maxAcc, _startSpeed,
                                     _endSpeed)) {}

std::optional<RobotInstant> TrapezoidalPath::eval(RJ::Seconds time) const {
    double distance;
    double speedOut;
    bool valid = TrapezoidalMotion(_pathLength,   // PathLength
                                   _maxSpeed,     // maxSpeed
                                   _maxAcc,       // maxAcc
                                   time.count(),  // time
                                   _startSpeed,   // startSpeed
                                   _endSpeed,     // endSpeed
                                   distance,      // posOut
                                   speedOut);     // speedOut
    if (!valid) return std::nullopt;

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
    // Check for valid arguments
    if (startTime < RJ::Seconds::zero()) {
        throw std::invalid_argument("TrapezoidalPath::subPath(): startTime(" +
                                    to_string(startTime) +
                                    ") can't be less than zero");
    }

    if (endTime < RJ::Seconds::zero()) {
        throw std::invalid_argument("TrapezoidalPath::subPath(): endTime(" +
                                    to_string(endTime) +
                                    ") can't be less than zero");
    }

    if (startTime > endTime) {
        throw std::invalid_argument(
            "TrapezoidalPath::subPath(): startTime(" + to_string(startTime) +
            ") can't be after endTime(" + to_string(endTime) + ")");
    }

    if (startTime >= _duration) {
        debugThrow(std::invalid_argument(
            "TrapezoidalPath::subPath(): startTime(" + to_string(startTime) +
            ") can't be greater than the duration(" + to_string(_duration) +
            ") of the path"));
        return std::make_unique<TrapezoidalPath>(Geometry2d::Point(0, 0), 0,
                                                 Geometry2d::Point(0, 0), 0,
                                                 _constraints);
    }

    std::optional<RobotInstant> start = evaluate(startTime);
    std::optional<RobotInstant> end = evaluate(endTime);

    // Start can return null when startTime < pathStartTime
    // It should be covered under the test for startTime > 0,
    // but this will fail gracefully in case something wasn't considered
    if (!start) {
        start = RobotInstant(
            MotionInstant(_startPos, _pathDirection * _startSpeed));
    }

    // End can return null when endTime > path_duration
    // This function should return the end of the path in that case,
    // not throw an error
    if (!end) {
        end = RobotInstant(MotionInstant(_endPos, _pathDirection * _endSpeed));
    }

    return std::make_unique<TrapezoidalPath>(
        start->motion.pos, start->motion.vel.mag(), end->motion.pos,
        end->motion.vel.mag(), _constraints);
}

}  // namespace Planning
