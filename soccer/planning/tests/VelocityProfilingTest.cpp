#include <gtest/gtest.h>

#include <fstream>

#include "planning/low_level/PathSmoothing.hpp"
#include "planning/low_level/VelocityProfiling.hpp"

using Geometry2d::Point;

namespace Planning {

TEST(VelocityProfiling, BasicTest) {
    MotionConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}, Point{2, 0}};
    BezierPath path(std::move(points), Point(1, 0), Point(1, 0), constraints);

    RJ::Time start_time{0s};

    Trajectory result = ProfileVelocity(path, 0.5, 0, constraints, start_time);

    std::ofstream out("/tmp/velocity.csv");
    for (auto cursor = result.cursor_begin(); cursor.has_value();
         cursor.advance(0.01s)) {
        RobotInstant instant = cursor.value();

        out << RJ::Seconds(instant.stamp - start_time) << ", "
            << instant.position().x() << ", " << instant.position().y() << ", "
            << instant.linear_velocity().x() << ", "
            << instant.linear_velocity().y() << ", "
            << instant.linear_velocity().mag() << std::endl;
    }
}

}  // namespace Planning