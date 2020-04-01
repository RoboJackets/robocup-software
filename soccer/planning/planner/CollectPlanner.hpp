#pragma once
#include "Planner.hpp"
#include "PathTargetPlanner.hpp"
namespace Planning {
enum class CollectState { Course, Fine, Control };
class CollectPlanner: public PlannerForCommandType<CollectCommand> {
public:
    CollectPlanner(): PlannerForCommandType<CollectCommand>("CollectPlanner"),
            _collectStates(Num_Shells, CollectState::Course) {}
    ~CollectPlanner() override = default;

    static void createConfiguration(Configuration* cfg);

    Trajectory plan(PlanRequest&& request);

private:
    // approach the ball from far away
    Trajectory course(PlanRequest&& request);
    // approach the ball at constant velocity until contact
    Trajectory fine(PlanRequest&& request);
    // stop without losing the ball
    Trajectory control(PlanRequest&& request);

    void processStateTransitions(const PlanRequest& request);

    // find the direction the robot will approach the ball (as a unit vector)
    Geometry2d::Point findApproachDirection(const PlanRequest& request) const {
        const Ball& ball = request.context->state.ball;
        return ball.vel.mag() < *_ballSpeedApproachDirectionCutoff ? (ball.pos - request.start.pose.position()).norm() : ball.vel.norm();
    }

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

    // How close to the ball do we have to be before transferring to the control
    // state This should be almost zero Increase if the noise on vision causes
    // problems and we never transition
    static ConfigDouble* _distCutoffToControl;  // m

    // Control acceleration controls the touch to stop
    // Lower this if we decelerate too quickly for the dribbler to keep a back
    // spin on
    static ConfigDouble* _controlAccelScalePercent;  // %

    // Controls at which ball speed we should try to go directly to the ball
    // or to move behind it and in the same direction as it
    //
    // Low number indicates that it will always try to choose a point for the
    // robot behind the velocity vector
    //
    // High number indicates that it will always try to choose a point nearest
    // to the current robot position
    static ConfigDouble* _ballSpeedApproachDirectionCutoff;  // m/s

    // the amount the target needs to change before replanning
    static ConfigDouble* _targetChangeThreshold;

    std::vector<CollectState> _collectStates;
    std::array<std::optional<RobotInstant>, Num_Shells> _courseTargetInstants;

    PathTargetPlanner _pathTargetPlanner;
};
}