#include "TrapezoidalPath.hpp"

namespace Planning {

TrapezoidalPath::TrapezoidalPath(Geometry2d::Point startPos, float startSpeed,
                                 Geometry2d::Point endPos, float endSpeed,
                                 const MotionConstraints& constraints)
    : startPos(startPos),
      startSpeed(startSpeed),
      endPos(endPos),
      endSpeed(endSpeed),
      pathLength((startPos - endPos).mag()),
      maxAcc(constraints.maxAcceleration),
      maxSpeed(constraints.maxSpeed),
      pathDirection((endPos - startPos).normalized()) {
    float minSpeed = maxSpeed;
    if (startSpeed < minSpeed) {
        startSpeed = minSpeed;
    }

    // Precalculate the duration of the path
    duration = Trapezoidal::getTime(pathLength,  // distance
                                    pathLength,  // pathLength
                                    maxSpeed, maxAcc, startSpeed, endSpeed);
}

boost::optional<MotionInstant> TrapezoidalPath::evaluate(float time) const {
    float distance;
    float speedOut;
    bool valid = TrapezoidalMotion(pathLength,  // PathLength
                                   maxSpeed,    // maxSpeed
                                   maxAcc,      // maxAcc
                                   time,        // time
                                   startSpeed,  // startSpeed
                                   endSpeed,    // endSpeed
                                   distance,    // posOut
                                   speedOut);   // speedOut
    if (!valid) return boost::none;

    return MotionInstant(pathDirection * distance + startPos,
                         pathDirection * speedOut);
}

bool TrapezoidalPath::hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                          float initialTime) const {
    for (Time t = initialTime * TimestampToSecs + startTime();
         t < startTime() + duration * SecsToTimestamp;
         t += 0.25 * SecsToTimestamp) {
        auto instant = evaluate((t - startTime()) * TimestampToSecs);
        if (instant) {
            for (auto& shape : obstacles.shapes()) {
                hitTime = t * TimestampToSecs;
                if (shape->hit(instant->pos)) return true;
            }
        }
    }

    return false;
}

std::unique_ptr<Path> TrapezoidalPath::subPath(float startTime,
                                               float endTime) const {
    debugThrow("This function is not implemented");
    return nullptr;
}

}  // namespace Planning
