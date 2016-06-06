#pragma once

#include "SingleRobotPathPlanner.hpp"
#include <Geometry2d/Point.hpp>

class Configuration;
class ConfigDouble;

namespace Planning {

class LineKickPlanner : public SingleRobotPathPlanner {
public:
    LineKickPlanner() : SingleRobotPathPlanner(false){};
    virtual std::unique_ptr<Path> run(SinglePlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Pivot;
    }

private:
    bool shouldReplan(const SinglePlanRequest& planRequest) const;
};

}  // namespace Planning
