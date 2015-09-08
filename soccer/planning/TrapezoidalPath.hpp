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
#include <protobuf/LogFrame.pb.h>

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
                    const MotionConstraints& constraints);

    virtual boost::optional<MotionInstant> evaluate(float time) const override;

    // TODO: only return true for *new* obstacles
    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float initialTime = 0) const override;

    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const override;

    virtual float getDuration() const override { return duration; }

    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const override;

    virtual boost::optional<MotionInstant> destination() const override {
        return MotionInstant(endPos, pathDirection * endSpeed);
    }

    virtual std::unique_ptr<Path> clone() const override {
        debugThrow("This function is not implemented");
        return nullptr;
    }
};

}  // namespace Planning
