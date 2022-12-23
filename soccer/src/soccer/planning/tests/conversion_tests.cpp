#include <gtest/gtest.h>

#include <rj_convert/testing/ros_convert_testing.hpp>

#include "planning/planner/motion_command.hpp"
#include "planning/trajectory.hpp"

using rj_geometry::Point;
using rj_geometry::Pose;
using rj_geometry::Twist;

namespace planning {

namespace testing {

// NOLINTNEXTLINE
TEST(RosConversions, EmptyMotionCommand) { test_lossless_convert_cpp_value(EmptyCommand{}); }

// NOLINTNEXTLINE
TEST(RosConversions, PathTargetCommand) {
    PathTargetCommand path_target{LinearMotionInstant(Point(1.0, 2.0), Point(3.0, 4.0)),
                                  TargetFacePoint{Point(1.0, 2.0)}};
    test_lossless_convert_cpp_value(path_target);
}

// NOLINTNEXTLINE
TEST(RosConversions, WorldVelCommand) {
    test_lossless_convert_cpp_value(WorldVelCommand{Point(1.0, 2.0)});
}

// NOLINTNEXTLINE
TEST(RosConversions, PivotCommand) {
    test_lossless_convert_cpp_value(PivotCommand{Point(1.0, 2.0), Point(3.0, 4.0)});
}

// NOLINTNEXTLINE
TEST(RosConversions, SettleCommand) {
    test_lossless_convert_cpp_value(SettleCommand{Point(1.0, 2.0)});
}

// NOLINTNEXTLINE
TEST(RosConversions, CollectCommand) { test_lossless_convert_cpp_value(CollectCommand{}); }

// NOLINTNEXTLINE
TEST(RosConversions, GoalieIdleCommand) { test_lossless_convert_cpp_value(GoalieIdleCommand{}); }

// NOLINTNEXTLINE
TEST(RosConversions, LineKickCommand) {
    test_lossless_convert_cpp_value(LineKickCommand{Point(1.0, 2.0)});
}

// NOLINTNEXTLINE
TEST(RosConversions, InterceptCommand) {
    test_lossless_convert_cpp_value(InterceptCommand{Point(1.0, 2.0)});
}

// NOLINTNEXTLINE
TEST(RosConversions, MotionCommand) {
    PathTargetCommand path_target{LinearMotionInstant(Point(1.0, 2.0), Point(3.0, 4.0)),
                                  TargetFacePoint{Point(1.0, 2.0)}};
    test_lossless_convert_cpp_value(MotionCommand{EmptyCommand{}});
    test_lossless_convert_cpp_value(MotionCommand{path_target});
    test_lossless_convert_cpp_value(MotionCommand{WorldVelCommand{Point(1.0, 2.0)}});
    test_lossless_convert_cpp_value(MotionCommand{PivotCommand{Point(1.0, 2.0), Point(3.0, 4.0)}});
    test_lossless_convert_cpp_value(MotionCommand{InterceptCommand{Point(1.0, 2.0)}});
}

// NOLINTNEXTLINE
TEST(RosConversions, Trajectory) {
    RJ::Time start;
    RobotInstant start_instant = RobotInstant(Pose(0, 0, 0), Twist(1, 0, 0), start);
    RobotInstant mid_instant = RobotInstant(Pose(1, 1, 3), Twist(1, 0, 0), start + 1s);
    RobotInstant end_instant = RobotInstant(Pose(2, 0, 6), Twist(1, 0, 0), start + 1500ms);

    Trajectory trajectory({start_instant, mid_instant, end_instant});
    // NOLINTNEXTLINE
    EXPECT_THROW(test_lossless_convert_cpp_value(trajectory), std::invalid_argument);

    trajectory.mark_angles_valid();
    // NOLINTNEXTLINE
    EXPECT_THROW(test_lossless_convert_cpp_value(trajectory), std::bad_optional_access);

    trajectory.stamp(start);
    test_lossless_convert_cpp_value(trajectory);
}

}  // namespace testing
}  // namespace planning
