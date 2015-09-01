#pragma once

#include "motion/TrapezoidalMotion.hpp"
#include "MotionConstraints.hpp"
#include "MotionInstant.hpp"
#include <Configuration.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Segment.hpp>
#include <planning/Path.hpp>

namespace Planning {

/**
 * @brief Represents a straight-line path with a trapezoidal velocity profile
 *
 * @details The path represents a function of position given time that the robot
 * should follow.
 * The path is made up of other Paths and can be made up of CompositePaths.
 */
class TrapezoidalPath : public Path {
private:
    const Geometry2d::Point startPos, endPos;
    const Geometry2d::Point pathDirection;
    const float startSpeed, endSpeed;

    const float pathLength;
    const float maxAcc;
    const float maxSpeed;

    float duration;

public:
    TrapezoidalPath(Geometry2d::Point startPos, float startSpeed,
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

    virtual bool valid() const override { return true; }

    virtual boost::optional<MotionInstant> evaluate(float time) const override {
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

    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float startTime = 0) const override {
        throw std::logic_error("This function is not implemented");
    }

    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const override {
        Packet::DebugPath* dbg = state->logFrame->add_debug_paths();
        dbg->set_layer(state->findDebugLayer(layer));
        *dbg->add_points() = startPos;
        *dbg->add_points() = endPos;
    }

    virtual float getDuration() const override { return duration; }

    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const override {
        debugThrow("This function is not implemented");
        return nullptr;
    }

    virtual boost::optional<MotionInstant> destination() const override {
        return MotionInstant(endPos, pathDirection * endSpeed);
    }
    virtual std::unique_ptr<Path> clone() const override {
        debugThrow("This function is not implemented");
        return nullptr;
    }
};

}  // namespace Planning
