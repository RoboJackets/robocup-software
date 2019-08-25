#pragma once

#include <optional>

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
    const double _startSpeed, _endSpeed;

    const double _pathLength;
    const double _maxAcc;
    const double _maxSpeed;
    const MotionConstraints _constraints;

    const RJ::Seconds _duration;

public:
    TrapezoidalPath(Geometry2d::Point startPos, double startSpeed,
                    Geometry2d::Point endPos, double endSpeed,
                    const MotionConstraints constraints);

    // TODO: only return true for *new* obstacles
    virtual bool hit(const Geometry2d::ShapeSet& obstacles,
                     RJ::Seconds initialTime,
                     RJ::Seconds* hitTime) const override;

    virtual RJ::Seconds getDuration() const override { return _duration; }

    double maxSpeed() const { return _maxSpeed; }

    virtual std::unique_ptr<Path> subPath(RJ::Seconds startTime,
                                          RJ::Seconds endTime) const override;

    virtual RobotInstant end() const override {
        return RobotInstant(MotionInstant(_endPos, _pathDirection * _endSpeed));
    }

    virtual RobotInstant start() const override {
        return RobotInstant(
            MotionInstant(_startPos, _pathDirection * _startSpeed));
    }

    virtual std::unique_ptr<Path> clone() const override {
        return std::make_unique<TrapezoidalPath>(
            _startPos, _startSpeed, _endPos, _endSpeed, _constraints);
    }

protected:
    virtual std::optional<RobotInstant> eval(RJ::Seconds time) const override;
};

}  // namespace Planning
