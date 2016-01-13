#include <gtest/gtest.h>
#include "TargetVelPathPlanner.hpp"
#include "planning/MotionCommand.hpp"
#include <Geometry2d/Point.hpp>

using namespace Geometry2d;

namespace Planning {

TEST(TargetVelPathPlannerTest, run) {
    MotionInstant startInstant({0, 0}, {0, 0});
    WorldVelTargetCommand cmd(Point(0, 1));

    MotionConstraints motionConstraints;
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Rect>(Point(-1, 5), Point(1, 4)));

    TargetVelPathPlanner planner;
    auto path = planner.run(startInstant, &cmd, motionConstraints, &obstacles);

    ASSERT_NE(nullptr, path) << "Planner returned null path";

    // Ensure that the path is obstacle-free
    float hitTime;
    EXPECT_FALSE(path->hit(obstacles, hitTime, 0))
        << "Returned path hits obstacles";

    // Ensure that the path moves in the direction of the target world velocity
    // (positive y-axis)
    boost::optional<RobotInstant> instant = path->evaluate(0.1);
    //ASSERT_NE(boost::none, instant);
    ASSERT_FALSE(instant);
    EXPECT_FLOAT_EQ(0, instant->motion.pos.x);
    EXPECT_GT(instant->motion.pos.y, 0);
}

}  // namespace Planning
