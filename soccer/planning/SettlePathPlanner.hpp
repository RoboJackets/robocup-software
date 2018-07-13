#pragma once

#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include "RRTPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"

namespace Planning {

/**
 * @brief Planner which tries to move around the ball to intercept it
 *
 * TODO: Clean up description
 */
class SettlePathPlanner : public SingleRobotPathPlanner {
public:
    SettlePathPlanner() : SingleRobotPathPlanner(false), rrtPlanner(0, 250){};
    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Settle;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    boost::optional<Geometry2d::Point> targetFinalCaptureDirectionPos;

    static ConfigDouble _ballSpeedPercentForDampen;
    static ConfigDouble _minSpeedToIntercept;
    static ConfigDouble _maxAngleOffBallForDampen;
    static ConfigDouble _searchStartTime;
    static ConfigDouble _searchEndTime;
    static ConfigDouble _searchIncTime;

    static double ballSpeedPercentForDampen() { return *_ballSpeedPercentForDampen; }
    static double minSpeedToIntercept() { return *_goalVelChangeThreshold; }
    static double maxAngleOffBallForDampen() { return *_maxAngleOffBallForDampen; }
    static double searchStartTime() { return *_searchStartTime; }
    static double searchEndTime() { return *_searchEndTime; }
    static double searchIncTime() { return *_searchIncTime; }
};

