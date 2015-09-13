#pragma once

#include <Geometry2d/Point.hpp>
#include <boost/optional.hpp>
#include "planning/MotionInstant.hpp"
#include "Utils.hpp"

namespace Planning {


class MotionCommand {
public:
    enum CommandType { PathTarget, WorldVel, Pivot };
    virtual ~MotionCommand() = default;
    CommandType getCommandType() const { return commandType; }
    virtual std::unique_ptr<Planning::MotionCommand> clone() const = 0;
protected:
    MotionCommand(const MotionCommand& that) = default;
    MotionCommand(CommandType command) : commandType(command) {}

private:
    // The type of command
    const CommandType commandType;
};

struct PathTargetCommand : public MotionCommand {
    virtual std::unique_ptr<Planning::MotionCommand> clone() const override {
        return std::make_unique<PathTargetCommand>(*this);
    }
    explicit PathTargetCommand(const MotionInstant& goal) : MotionCommand(MotionCommand::PathTarget), pathGoal(goal) {};
    MotionInstant pathGoal;
};

struct WorldVelTargetCommand : public MotionCommand {
    explicit WorldVelTargetCommand(Geometry2d::Point vel) : MotionCommand(MotionCommand::WorldVel), worldVel(vel) {};
    virtual std::unique_ptr<Planning::MotionCommand> clone() const override {
        return std::make_unique<WorldVelTargetCommand>(*this);
    }
    Geometry2d::Point worldVel;
};
struct PivotCommand : public MotionCommand {
    explicit PivotCommand(Geometry2d::Point target) : MotionCommand(MotionCommand::Pivot), pivotTarget(target) {};
    virtual std::unique_ptr<Planning::MotionCommand> clone() const override {
        return std::make_unique<PivotCommand>(*this);
    }
    Geometry2d::Point pivotTarget;
};

}  // namespace Planning
