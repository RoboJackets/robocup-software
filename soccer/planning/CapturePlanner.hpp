#pragma once

#include <Geometry2d/Point.hpp>
#include "RRTPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"
class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * Planner which tries to move around the ball to intercept it then capture it
 * 
 * TODO: Clean up description
 */
class CapturePlanner : public SingleRobotPathPlanner {
public:
    CapturePlanner() : SingleRobotPathPlanner(false), rrtPlanner(0, 250){};
    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand:CommandType commandType() const override {
        return MotionCommand::Capture;
    }

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    boost::optional<Geometry2d::Point> targetFinalCaptureDirectionPos;
};
}