#include <gtest/gtest.h>

#include <rj_convert/testing/ros_convert_testing.hpp>

#include "planning/planner/motion_command.hpp"

using rj_geometry::Point;
using rj_geometry::Pose;
using rj_geometry::Twist;

namespace planning {

bool operator==(const LinearMotionInstant& a, const LinearMotionInstant& b) {
    return a.velocity == b.velocity && a.position == b.position;
}

bool operator==([[maybe_unused]] const TargetFaceTangent& a,
                [[maybe_unused]] const TargetFaceTangent& b) {
    return true;
}
bool operator==(const TargetFaceAngle& a, const TargetFaceAngle& b) { return a.target == b.target; }
bool operator==(const TargetFacePoint& a, const TargetFacePoint& b) {
    return a.face_point == b.face_point;
}

bool operator==([[maybe_unused]] const EmptyCommand& a, [[maybe_unused]] const EmptyCommand& b) {
    return true;
}
bool operator==(const PathTargetCommand& a, const PathTargetCommand& b) {
    return a.goal == b.goal && a.angle_override == b.angle_override;
}
bool operator==(const WorldVelCommand& a, const WorldVelCommand& b) {
    return a.world_vel == b.world_vel;
}
bool operator==(const PivotCommand& a, const PivotCommand& b) {
    return a.pivot_target == b.pivot_target && a.pivot_point == b.pivot_point;
}
bool operator==(const SettleCommand& a, const SettleCommand& b) { return a.target == b.target; }
bool operator==([[maybe_unused]] const CollectCommand& a,
                [[maybe_unused]] const CollectCommand& b) {
    return true;
}
bool operator==(const LineKickCommand& a, const LineKickCommand& b) { return a.target == b.target; }
bool operator==(const InterceptCommand& a, const InterceptCommand& b) {
    return a.target == b.target;
}
bool operator==(const Trajectory& a, const Trajectory& b) {
    // Don't check debug text, as that doesn't get converted over ROS
    return a.instants() == b.instants() && a.time_created() == b.time_created() &&
           a.angles_valid() == b.angles_valid();
}

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