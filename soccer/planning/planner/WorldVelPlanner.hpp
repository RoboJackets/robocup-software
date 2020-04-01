#include "MotionCommand.hpp"
#include "PathTargetPlanner.hpp"
#include "Planner.hpp"
#include "planning/trajectory/Trajectory.hpp"
namespace Planning {
class WorldVelPlanner : public PlannerForCommandType<WorldVelCommand> {
public:
    WorldVelPlanner() : PlannerForCommandType("WorldVelPlanner") {}
    Trajectory plan(PlanRequest&& request) {
        using namespace Geometry2d;
        Point targetVel =
            std::get<WorldVelCommand>(request.motionCommand).worldVel;
        Point targetPoint =
            request.start.pose.position() + targetVel.normalized(.2);
        request.constraints.mot.maxSpeed = targetVel.mag();
        RobotInstant goalInstant{Pose{targetPoint, 0}, Twist{targetVel, 0},
                                 RJ::Time{0}};
        request.motionCommand = PathTargetCommand{goalInstant};
        return _pathTargetPlanner(std::move(request));
    }

private:
    PathTargetPlanner _pathTargetPlanner;
};
}  // namespace Planning