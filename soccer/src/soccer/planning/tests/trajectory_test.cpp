#include <gtest/gtest.h>

#include <rrt/planning/Path.hpp>

#include "testing_utils.hpp"
#include "math.h"
#include "planning/instant.hpp"
#include "planning/trajectory.hpp"
#include "planning/planner/path_target_planner.hpp"
#include "planning/primitives/path_smoothing.hpp"
#include "planning/primitives/rrt_util.hpp"
#include "planning/primitives/velocity_profiling.hpp"

using namespace planning;
using namespace rj_geometry;
using namespace planning::TestingUtils;

TEST(Trajectory, Interpolation) {
    // Test the basics of the Trajectory class, including interpolation, instant
    // addition/insertion, and duration calculations.
    RJ::Time start = RJ::now();

    RobotInstant start_instant = RobotInstant(Pose(0, 0, 0), Twist(1, 0, 0), start);
    RobotInstant mid_instant = RobotInstant(Pose(1, 1, 3), Twist(1, 0, 0), start + 1s);
    RobotInstant end_instant = RobotInstant(Pose(2, 0, 6), Twist(1, 0, 0), start + 1500ms);

    Trajectory trajectory;
    trajectory.append_instant(start_instant);
    trajectory.append_instant(mid_instant);

    ASSERT_EQ(*trajectory.evaluate(start), start_instant);
    ASSERT_EQ(*trajectory.evaluate(trajectory.end_time()), mid_instant);
    EXPECT_FALSE(trajectory.evaluate(trajectory.begin_time() - RJ::Seconds(1e-3s)));
    EXPECT_FALSE(trajectory.evaluate(trajectory.end_time() + RJ::Seconds(1e-3s)));

    EXPECT_FALSE(Trajectory{{}}.evaluate(RJ::Seconds(0s)));
    EXPECT_FALSE(Trajectory{{}}.evaluate(start));

    Twist mid_twist;
    {
        RobotInstant instant = *trajectory.evaluate(start + 500ms);
        EXPECT_NEAR(instant.position().x(), 0.5, 1e-6);
        EXPECT_NEAR(instant.position().y(), 0.5, 1e-6);
        EXPECT_NEAR(instant.pose.heading(), 1.5, 1e-6);
        mid_twist = instant.velocity;
    }

    // Make sure we use the right segment.
    trajectory.append_instant(end_instant);
    {
        RobotInstant instant = *trajectory.evaluate(start + 1250ms);
        EXPECT_NEAR(instant.position().x(), 1.5, 1e-6);
        EXPECT_NEAR(instant.position().y(), 0.5, 1e-6);
        EXPECT_NEAR(instant.pose.heading(), 4.5, 1e-6);

        // Twist should be the same as halfway through the other segment, but
        // rescaled because this segment only lasts 500ms.
        EXPECT_NEAR(-mid_twist.linear().y() * 2, instant.velocity.linear().y(), 1e-6);
        EXPECT_NEAR(mid_twist.angular() * 2, instant.velocity.angular(), 1e-6);
    }

    EXPECT_EQ(*trajectory.evaluate(trajectory.end_time()), end_instant);

    EXPECT_TRUE(trajectory.check_time(start + 500ms));
    EXPECT_FALSE(trajectory.check_time(start - 500ms));
    EXPECT_FALSE(trajectory.check_time(start + 2500ms));
    EXPECT_TRUE(trajectory.check_seconds(500ms));
    EXPECT_FALSE(trajectory.check_seconds(-500ms));
    EXPECT_FALSE(trajectory.check_seconds(2500ms));

    EXPECT_EQ(trajectory.duration(), RJ::Seconds(end_instant.stamp - start_instant.stamp));
}

TEST(Trajectory, TrajectoryCursor) {
    std::vector<RobotInstant> instants;
    RobotInstant a{Pose{{1, 1}, 1}, Twist{}, RJ::Time(1s)};
    RobotInstant b{Pose{{2, 2}, 2}, Twist{}, RJ::Time(2s)};
    RobotInstant c{Pose{{5, 5}, 5}, Twist{}, RJ::Time(5s)};
    instants.push_back(a);
    instants.push_back(b);
    instants.push_back(c);
    Trajectory traj{std::move(instants)};

    auto cursor = traj.cursor_begin();
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), a));
    cursor.next_knot();
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), b));
    cursor.advance(3s);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), c));

    cursor.seek(c.stamp);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), c));
    cursor.seek(b.stamp);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), b));
    cursor.seek(a.stamp);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), a));

    // Make sure we can advance multiple steps forwards.
    cursor.advance(4s);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), c));

    cursor.seek(a.stamp);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), a));
    cursor.advance(0.5s);
    cursor.advance(0.5s);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), b));
    cursor.advance(0.5s);
    cursor.advance(1.5s);
    cursor.advance(1.0s);
    ASSERT_TRUE(cursor.has_value());
    EXPECT_TRUE(RobotInstant::nearly_equals(cursor.value(), c));
    cursor.advance(0.1s);
    EXPECT_FALSE(cursor.has_value());
}

TEST(Trajectory, BezierPath) {
    std::vector<Point> points = {Point(0, 0), Point(1, 0.5), Point(1.5, 1), Point(2, 2)};

    Point vi(1, 0);
    Point vf(0, 1);

    MotionConstraints constraints;
    constraints.max_speed = 3.0;
    constraints.max_acceleration = 3.0;

    planning::BezierPath path(points, vi, vf, constraints);

    for (int i = 0; i < 4; i++) {
        Point p;
        Point v;
        double k = 0;
        path.evaluate(i / 3.0, &p, &v, &k);

        std::cout << p << ", " << v << std::endl;
        EXPECT_NEAR((p - points[i]).mag(), 0, 1e-6);
        if (i == 0) {
            EXPECT_NEAR(v.angle_between(vi), 0, 1e-3);
        }
        if (i == 3) {
            EXPECT_NEAR(v.angle_between(vf), 0, 1e-3);
        }
    }
}

TEST(Trajectory, SubTrajectory) {
    std::vector<RobotInstant> instants;
    RobotInstant a{Pose{{0, 0}, 0}, Twist{{1, 0}, 0}, RJ::Time(0s)};
    RobotInstant b{Pose{{1, 1}, 0}, Twist{{2, 0}, 1}, RJ::Time(2s)};
    RobotInstant c{Pose{{2, 0}, 0}, Twist{{1, 0}, 1}, RJ::Time(4s)};
    instants.push_back(a);
    instants.push_back(b);
    instants.push_back(c);
    Trajectory traj{std::move(instants)};

    {
        RJ::Time t0{1s};
        RJ::Time t1{3s};
        RobotInstant ab = traj.evaluate(t0).value();
        RobotInstant bc = traj.evaluate(t1).value();

        Trajectory sub = traj.sub_trajectory(t0, t1);
        EXPECT_TRUE(Trajectory::nearly_equal(sub, Trajectory{{ab, b, bc}}));
    }

    {
        RJ::Time t0{3s};
        RJ::Time t1{6s};
        RobotInstant bc = traj.evaluate(t0).value();

        Trajectory sub = traj.sub_trajectory(t0, t1);
        EXPECT_TRUE(Trajectory::nearly_equal(sub, Trajectory{{bc, c}}));
    }

    {
        RJ::Time t0{4s};
        RJ::Time t1{6s};
        RobotInstant bc = traj.evaluate(t0).value();

        Trajectory sub = traj.sub_trajectory(t0, t1);
        EXPECT_TRUE(Trajectory::nearly_equal(sub, Trajectory{{c}}));
    }
}

TEST(Trajectory, SubTrajectoryEndpoints) {
    RobotInstant a{Pose{{0, 0}, 0}, Twist{{1, 0}, 0}, RJ::Time(0s)};
    RobotInstant b{Pose{{1, 1}, 0}, Twist{{2, 0}, 1}, RJ::Time(2s)};
    RobotInstant c{Pose{{2, 0}, 0}, Twist{{1, 0}, 1}, RJ::Time(4s)};
    Trajectory traj{{a, b, c}};

    Trajectory sub = traj.sub_trajectory(RJ::Time(0s), RJ::Time(4s));
    EXPECT_TRUE(Trajectory::nearly_equal(sub, Trajectory{{a, b, c}}));
}

TEST(Trajectory, Combining) {
    std::vector<RobotInstant> instants;
    RobotInstant a{Pose{{0, 0}, 0}, Twist{{1, 0}, 0}, RJ::Time(0s)};
    RobotInstant b{Pose{{1, 1}, 0}, Twist{{2, 0}, 1}, RJ::Time(2s)};
    RobotInstant c{Pose{{2, 0}, 0}, Twist{{1, 0}, 1}, RJ::Time(4s)};
    Trajectory traj_1{{a, b}};
    Trajectory traj_2{{b, c}};

    Trajectory combined(std::move(traj_1), traj_2);
    EXPECT_TRUE(traj_1.empty()) << "Should have moved out of the first trajectory";
    EXPECT_TRUE(Trajectory::nearly_equal(combined, Trajectory{{a, b, c}}));
}

TEST(Trajectory, CombiningFail) {
    std::vector<RobotInstant> instants;
    RobotInstant a{Pose{{0, 0}, 0}, Twist{{1, 0}, 0}, RJ::Time(0s)};
    RobotInstant b1{Pose{{1, 1}, 0}, Twist{{2, 0}, 1}, RJ::Time(2s)};
    RobotInstant b2{Pose{{1, 0.5}, 0}, Twist{{2, 0}, 1}, RJ::Time(2s)};
    RobotInstant c{Pose{{2, 0}, 0}, Twist{{1, 0}, 1}, RJ::Time(4s)};
    Trajectory traj_1{{a, b1}};
    Trajectory traj_2{{b2, c}};

    EXPECT_THROW((Trajectory{std::move(traj_1), traj_2}), std::invalid_argument);
}
