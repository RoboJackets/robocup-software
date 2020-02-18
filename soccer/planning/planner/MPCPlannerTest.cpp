#include <gtest/gtest.h>
#include "MPCPlanner.h"

using Geometry2d::Pose;
using Geometry2d::Twist;
using Geometry2d::Point;

namespace Planning {

namespace test {

TEST(MPC, SimpleTest) {
    RobotInstant start = {
        Pose(Point(0, 0), 0),
        Twist(Point(0, 0), 0),
        RJ::now()
    };

    MotionCommand command = PosVelCommand {
        Pose(Point(3, 3), 0),
        Twist(Point(0, 1.5), 0)
    };

    Geometry2d::ShapeSet static_obstacles;

    MPCPlanner planner;
    PlanRequest request(
            nullptr, start, command, RobotConstraints(),
            Trajectory({}), static_obstacles, {}, 0, 0);
    Trajectory trajectory = planner.plan(std::move(request));
    for (RJ::Time t = trajectory.begin_time(); t < trajectory.end_time(); t = t + RJ::Seconds(0.1)) {
        std::cout << trajectory.evaluate(t)->pose << "\t\t" << trajectory.evaluate(t)->velocity << std::endl;
    }
}

}

} // namespace Planning
