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
 *     should follow. The path is made up of other Paths and can be made up of
 *     CompositePaths.
 */
class TrapezoidalPath : public Path {
private:
    const Geometry2d::Point _startPos, _endPos;
    const Geometry2d::Point _pathDirection;
    const float _startSpeed, _endSpeed;

    const float _pathLength;
    const float _maxAcc;
    const float _maxSpeed;

    const float _duration;

public:
    TrapezoidalPath(Geometry2d::Point startPos, float startSpeed,
                    Geometry2d::Point endPos, float endSpeed,
                    const MotionConstraints& constraints);

    virtual boost::optional<RobotInstant> evaluate(float time) const override;

    // TODO: only return true for *new* obstacles
    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float initialTime = 0) const override;

    virtual float getDuration() const override { return _duration; }

    float maxSpeed() const { return _maxSpeed; }

    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const override;

    virtual RobotInstant end() const override {
        return RobotInstant(MotionInstant(_endPos, _pathDirection * _endSpeed));
    }

    virtual RobotInstant start() const override {
        return RobotInstant(MotionInstant(_startPos, _pathDirection * _startSpeed));
    }

    virtual std::unique_ptr<Path> clone() const override {
        debugThrow("This function is not implemented");
        return nullptr;
    }
};

}  // namespace Planning
