#include <gtest/gtest.h>

#include <rj_convert/testing/ros_convert_testing.hpp>

#include "planning/planner/motion_command.hpp"
#include "planning/trajectory.hpp"

using rj_geometry::Point;
using rj_geometry::Pose;
using rj_geometry::Twist;

namespace planning {

namespace testing {

// TODO(Kevin): add conversion test for new MotionCommand

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, EmptyMotionCommand) { test_lossless_convert_cpp_value(EmptyMotionCommand{}); } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, PathTargetMotionCommand) { */
/*     PathTargetMotionCommand path_target{LinearMotionInstant(Point(1.0, 2.0), Point(3.0, 4.0)), */
/*                                         FacePoint{Point(1.0, 2.0)}}; */
/*     test_lossless_convert_cpp_value(path_target); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, WorldVelMotionCommand) { */
/*     test_lossless_convert_cpp_value(WorldVelMotionCommand{Point(1.0, 2.0)}); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, PivotMotionCommand) { */
/*     test_lossless_convert_cpp_value(PivotMotionCommand{Point(1.0, 2.0), Point(3.0, 4.0)}); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, SettleMotionCommand) { */
/*     test_lossless_convert_cpp_value(SettleMotionCommand{Point(1.0, 2.0)}); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, CollectMotionCommand) { */
/*     test_lossless_convert_cpp_value(CollectMotionCommand{}); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, GoalieIdleMotionCommand) { */
/*     test_lossless_convert_cpp_value(GoalieIdleMotionCommand{}); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, LineKickMotionCommand) { */
/*     test_lossless_convert_cpp_value(LineKickMotionCommand{Point(1.0, 2.0)}); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, InterceptMotionCommand) { */
/*     test_lossless_convert_cpp_value(InterceptMotionCommand{Point(1.0, 2.0)}); */
/* } */

/* // NOLINTNEXTLINE */
/* TEST(RosConversions, MotionCommand) { */
/*     PathTargetMotionCommand path_target{LinearMotionInstant(Point(1.0, 2.0), Point(3.0, 4.0)), */
/*                                         FacePoint{Point(1.0, 2.0)}}; */
/*     test_lossless_convert_cpp_value(MotionCommand{EmptyMotionCommand{}}); */
/*     test_lossless_convert_cpp_value(MotionCommand{path_target}); */
/*     test_lossless_convert_cpp_value(MotionCommand{WorldVelMotionCommand{Point(1.0, 2.0)}}); */
/*     test_lossless_convert_cpp_value( */
/*         MotionCommand{PivotMotionCommand{Point(1.0, 2.0), Point(3.0, 4.0)}}); */
/*     test_lossless_convert_cpp_value(MotionCommand{InterceptMotionCommand{Point(1.0, 2.0)}}); */
/* } */

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
