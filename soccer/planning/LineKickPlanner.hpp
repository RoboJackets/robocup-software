#pragma once

#include "SingleRobotPathPlanner.hpp"
#include <Geometry2d/Point.hpp>
#include "RRTPlanner.hpp"
class Configuration;
class ConfigDouble;

namespace Planning {

class LineKickPlanner : public SingleRobotPathPlanner {
public:
    LineKickPlanner() : SingleRobotPathPlanner(false), rrtPlanner(250){};
    virtual std::unique_ptr<Path> run(SinglePlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::LineKick;
    }

private:
    bool shouldReplan(const SinglePlanRequest& planRequest) const;
    RRTPlanner rrtPlanner;

    //boost::optional<std::vector<CubicBezierControlPoints>> approachBezier;
    //std::unique_ptr<InterpolatedPath> approachPath;
    bool finalApproach=false;
    boost::optional<Geometry2d::Point>  targetKickPos;
};

}  // namespace Planning
