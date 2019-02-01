#pragma once

#include "RRTPlanner.hpp"
#include "DirectTargetPathPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * @brief Planner that tries to move onto and gain control of a slow moving ball
 * 
 * TODO: Clean up description 
 */
class CollectPathPlanner : public SingleRobotPathPlanner {
public:
    enum CollectPathPlannerStates {
        Approach,
        Control,
        Complete
    };

    CollectPathPlanner() : SingleRobotPathPlanner(false), rrtPlanner(0, 250), directPlanner(),
                           currentState(Approach), velocityTarget(0,0), firstTargetFound(false){};

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Collect;
    }

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    DirectTargetPathPlanner directPlanner;

    CollectPathPlannerStates currentState;

    // Ball Velocity Filtering Variables
    Geometry2d::Point velocityTarget;
    bool firstTargetFound;
};
}