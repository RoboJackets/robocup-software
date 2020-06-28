#include <gtest/gtest.h>

#include <fstream>

#include "planning/low_level/PathSmoothing.hpp"
#include "planning/low_level/VelocityProfiling.hpp"
#include "planning/tests/TestingUtils.hpp"

using Geometry2d::Point;

namespace Planning {

[[maybe_unused]]
void dump_trajectory(const std::string& filename, const Trajectory& trajectory) {
    std::ofstream out(filename);
    for (auto cursor = trajectory.cursor_begin(); cursor.has_value();
         cursor.advance(0.01s)) {
        RobotInstant instant = cursor.value();

        out << RJ::Seconds(instant.stamp - trajectory.begin_time()) << ", "
            << instant.position().x() << ", " << instant.position().y() << ", "
            << instant.linear_velocity().x() << ", "
            << instant.linear_velocity().y() << ", "
            << instant.linear_velocity().mag() << std::endl;
    }
}

TEST(VelocityProfiling, BasicTest) {
    RobotConstraints constraints;
    std::vector<Point> points{Point{0, 0}, Point{1, 1}, Point{2, 0}};
    BezierPath path(std::move(points), Point(1, 0), Point(1, 0), constraints.mot);

    RJ::Time start_time{0s};

    Trajectory result = ProfileVelocity(path, 0.5, 0, constraints.mot, start_time);

    TestingUtils::checkTrajectoryContinuous(result, constraints);
}

TEST(VelocityProfiling, ComplexPath) {
    RobotConstraints constraints;
    std::vector<Point> points{
        Point{0, 0},
        Point{1, 1},
        Point{2, 0},
        Point{1, -1},
        Point{0, -2},
        Point{2, -2}
    };
    BezierPath path(std::move(points), Point(1, 0), Point(1, 0), constraints.mot);

    RJ::Time start_time{0s};

    Trajectory result = ProfileVelocity(path, 0.5, 0, constraints.mot, start_time);

    TestingUtils::checkTrajectoryContinuous(result, constraints);
}

}  // namespace Planning