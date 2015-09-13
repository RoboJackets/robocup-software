#include <gtest/gtest.h>
#include "EscapeObstaclesPathPlanner.hpp"
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Circle.hpp>

using namespace Geometry2d;

namespace Planning {

TEST(EscapeObstaclesPathPlanner, run) {
    // The robot is at the origin
    MotionInstant startInstant({0, 0}, {0, 0});
    EmptyCommand cmd;  // "None" command

    // Add an circle of radius 5 centered at the origin as an obstacle
    ShapeSet obstacles;
    const float circleRadius = 5;
    obstacles.add(std::make_shared<Circle>(Point(0, 0), circleRadius));

    EscapeObstaclesPathPlanner planner;
    auto path = planner.run(startInstant, &cmd, MotionConstraints(), &obstacles);

    ASSERT_NE(nullptr, path) << "Planner returned null path";

    // Ensure that the path escapes the obstacle
    float hitTime;
    EXPECT_FALSE(path->hit(obstacles, hitTime, 0))
        << "Returned path hits obstacles";

    // Make sure the path's endpoint is close to the original point.  It
    // shouldn't be further than two steps outside of the closest possible
    // point.
    const float stepSize = EscapeObstaclesPathPlanner::stepSize();
    const float pathLength = (path->end().pos - startInstant.pos).mag();
    EXPECT_LE(pathLength, circleRadius + Robot_Radius + stepSize * 2)
        << "Path is longer than it should be";
}

}  // namespace Planning
