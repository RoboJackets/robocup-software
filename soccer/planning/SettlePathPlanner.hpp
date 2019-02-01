#pragma once

#include "RRTPlanner.hpp"
#include "DirectTargetPathPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * @brief Planner which tries to move around the ball to intercept it
 *
 * TODO: Clean up description
 */
class SettlePathPlanner : public SingleRobotPathPlanner {
public:
    enum SettlePathPlannerStates {
        Intercept,
        Dampen,
        Complete
    };

    SettlePathPlanner() : SingleRobotPathPlanner(false), rrtPlanner(0, 250), directPlanner(),
                          interceptTarget(0,0), firstTargetPointFound(false), currentState(Intercept) {};

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Settle;
    }

    // static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    DirectTargetPathPlanner directPlanner;
    boost::optional<Geometry2d::Point> targetFinalCaptureDirectionPos;

    SettlePathPlannerStates currentState;

    // Intercept Target Filtering Variables
    Geometry2d::Point interceptTarget;
    RJ::Seconds averagePathTime;
    bool firstTargetPointFound;

    // static ConfigDouble* _ballSpeedPercentForDampen;
    // static ConfigDouble* _minSpeedToIntercept;
    // static ConfigDouble* _maxAngleOffBallForDampen;
    // static ConfigDouble* _searchStartTime;
    // static ConfigDouble* _searchEndTime;
    // static ConfigDouble* _searchIncTime;

    // static double ballSpeedPercentForDampen = (double) *_ballSpeedPercentForDampen;
    // static double minSpeedToIntercept = (double) *_minSpeedToIntercept;
    // static double maxAngleOffBallForDampen = (double) *_maxAngleOffBallForDampen;
    // static double searchStartTime = (double) *_searchStartTime;
    // static double searchEndTime = (double) *_searchEndTime;
    // static double searchIncTime = (double) *_searchIncTime;
};
}
