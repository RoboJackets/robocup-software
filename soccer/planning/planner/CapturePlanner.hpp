#pragma once

#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/Planner.hpp"
#include "planning/planner/PathTargetPlanner.hpp"
/*
 * Plans a trajectory to the ball. After we reach the ball, it's up to Gameplay
 * to figure out what to do next (i.e. pass, shoot, pivot, etc.)
 */
namespace Planning {
class CapturePlanner: public PlannerForCommandType<CaptureCommand> {
public:
    Trajectory plan(PlanRequest &&request) override;
    std::string name() const { return "CapturePlanner"; };

    static void createConfiguration(Configuration* cfg);

private:
    Geometry2d::Point projectPointIntoField(Geometry2d::Point targetPoint, const Geometry2d::Rect& fieldRect, Geometry2d::Point ballPoint) const;

    std::optional<Trajectory> attemptCapture(const PlanRequest& request, double distAlongBallLine) const;

    /*
     * finds a goal point by brute force along the ball path
     * this method assumes final velocity of 0 which
     */
    Trajectory bruteForceCapture(const PlanRequest& request) const;

    //endpoints used for searching along the ball path
    static ConfigDouble* _searchStartDist; // m
    static ConfigDouble* _searchEndDist; // m
    static ConfigDouble* _searchIncDist; // m

    // if the ball speed is less than this cutoff then move directly at it;
    // otherwise approach along the ball line
    static ConfigDouble* _maxBallSpeedDirect; // m/s

    // When trying to reach the targetFacePoint it might be more efficient
    // to approach the ball at an angle rather than always staying orthogonal
    // this is the maximum angle away from orthogonal we will go
    static ConfigDouble* _maxApproachAngle; // radians

    // the maximum speed we will try to outrun a ball during a Collect behavior
    // (as a percent of MotionConstraints.maxSpeed)
    static ConfigDouble* _maxOutrunBallSpeedPercent; // %

    //the maximum acceleration while stopping with the ball.
    // (as a percent of MotionConstraints.maxAcceleration)
    static ConfigDouble* _ballContactAccelPercent; // %

    // At what speed should we be when we touch the ball (Well, the difference
    // in speed between the ball and robot) Should be as low as possible where
    // we still are able to touch the ball and control it If we are slamming
    // into the ball decrease this number If we aren't even touching it to the
    // dribbler, increase this number
    static ConfigDouble* _touchDeltaSpeed; // m/s

    // buffer distance for when we contact the ball. between these buffers, the
    // robot will move at a constant velocity
    static ConfigDouble* _bufferDistBeforeContact;
    static ConfigDouble* _bufferDistAfterContact;

    // amount of the previous trajectory that is kept during a partial replan.
    static ConfigDouble* _partialReplanLeadTime; // s

    // percent of the ball vel we match during dampen (when we catch a ball
    // moving fast toward us)
    static ConfigDouble* _ballSpeedPercentForDampen; // %
};
}