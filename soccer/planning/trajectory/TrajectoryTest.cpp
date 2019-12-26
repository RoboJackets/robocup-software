#include <gtest/gtest.h>
#include "Trajectory.hpp"
#include "PathSmoothing.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "VelocityProfiling.hpp"
#include "planning/planner/PathTargetPlanner.hpp"

using Planning::Trajectory;
using Planning::RobotInstant;
using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

TEST(Trajectory, Interpolation) {
    // Test the basics of the Trajectory class, including interpolation, instant
    // addition/insertion, and duration calculations.
    RJ::Time start = RJ::now();

    RobotInstant start_instant =
            Planning::RobotInstant(Pose(0, 0, 0), Twist(1, 0, 0), start);
    RobotInstant mid_instant =
            Planning::RobotInstant(Pose(1, 1, 3), Twist(1, 0, 0), start + 1s);
    RobotInstant end_instant =
            Planning::RobotInstant(Pose(2, 0, 6), Twist(1, 0, 0), start + 1500ms);

    Trajectory trajectory({});
    trajectory.AppendInstant(start_instant);
    trajectory.AppendInstant(mid_instant);

    EXPECT_EQ(*trajectory.evaluate(start), start_instant);
    EXPECT_EQ(*trajectory.evaluate(trajectory.end_time()), mid_instant);

    Twist mid_twist;
    {
        RobotInstant instant = *trajectory.evaluate(start + 500ms);
        EXPECT_NEAR(instant.pose.position().x(), 0.5, 1e-6);
        EXPECT_NEAR(instant.pose.position().y(), 0.5, 1e-6);
        EXPECT_NEAR(instant.pose.heading(), 1.5, 1e-6);
        mid_twist = instant.velocity;
    }

    // Make sure we use the right segment.
    trajectory.AppendInstant(end_instant);
    {
        RobotInstant instant = *trajectory.evaluate(start + 1250ms);
        EXPECT_NEAR(instant.pose.position().x(), 1.5, 1e-6);
        EXPECT_NEAR(instant.pose.position().y(), 0.5, 1e-6);
        EXPECT_NEAR(instant.pose.heading(), 4.5, 1e-6);

        // Twist should be the same as halfway through the other segment, but
        // rescaled because this segment only lasts 500ms.
        EXPECT_NEAR(-mid_twist.linear().y() * 2, instant.velocity.linear().y(), 1e-6);
        EXPECT_NEAR(mid_twist.angular() * 2, instant.velocity.angular(), 1e-6);
    }

    EXPECT_EQ(*trajectory.evaluate(trajectory.end_time()), end_instant);

    EXPECT_TRUE(trajectory.CheckTime(start + 500ms));
    EXPECT_FALSE(trajectory.CheckTime(start - 500ms));
    EXPECT_FALSE(trajectory.CheckTime(start + 2500ms));
    EXPECT_TRUE(trajectory.CheckSeconds(500ms));
    EXPECT_FALSE(trajectory.CheckSeconds(-500ms));
    EXPECT_FALSE(trajectory.CheckSeconds(2500ms));

    EXPECT_EQ(trajectory.duration(), RJ::Seconds(end_instant.stamp - start_instant.stamp));

    Trajectory trajectory_2({});
    trajectory_2.InsertInstant(start_instant);
    trajectory_2.InsertInstant(end_instant);
    trajectory_2.InsertInstant(mid_instant);

    EXPECT_EQ(trajectory.duration(), trajectory_2.duration());
    EXPECT_EQ(trajectory.instant(1), trajectory_2.instant(1));
}

TEST(PathSmoothing, PathMatches) {
    std::vector<Point> points = {
        Point(0, 0),
        Point(1, 0.5),
        Point(1.5, 1),
        Point(2, 2)
    };

    Point vi(1, 0), vf(0, 1);

    MotionConstraints constraints;
    constraints.maxSpeed = 3.0;
    constraints.maxAcceleration = 3.0;

    Planning::BezierPath path(points, vi, vf, constraints);

    {
        Point p, v;
        double k;
        path.Evaluate(0, &p, &v, &k);

        std::cout << p << ", " << v << std::endl;
        EXPECT_NEAR((p - points[0]).mag(), 0, 1e-6);
        EXPECT_NEAR(v.angleBetween(vi), 0, 1e-3);
    }
}

TEST(VelocityProfiling, Linear) {
    //todo
    ASSERT_TRUE(true);
}
TEST(VelocityProfiling, Anglular) {
    using namespace Geometry2d;
    using namespace Planning;
    using namespace std;

    MotionConstraints mot;
    RotationConstraints rot;
    RobotInstant start{Pose{}, Twist{}, RJ::now()};
    RobotInstant mid{Pose{Point{1,1}, M_PI/2}, Twist{Point{0,1}, 0}, RJ::now()};
    RobotInstant end{Pose{Point{2,2}, M_PI/2}, Twist{Point{1,0}, 0}, RJ::now()};
    ShapeSet obs;
    Trajectory preTraj = RRTTrajectory(start, mid, mot, obs);
    ASSERT_FALSE(preTraj.empty());
    RJ::Time t0 = preTraj.begin_time();
    AngleFunction angleFn = [t0](const RobotInstant& instant) -> double {
        return RJ::Seconds(instant.stamp - t0).count();
    };
    PlanAngles(preTraj, start, angleFn, rot);
    Trajectory postTraj = RRTTrajectory(preTraj.last(), end, mot, obs);
    ASSERT_FALSE(postTraj.empty());
    PlanAngles(postTraj, preTraj.last(), angleFn, rot);
    Trajectory combo{preTraj, postTraj};

    RJ::Seconds duration = combo.duration();
    function<void(const RobotInstant&)> printRobotInstant =
            [&combo](const RobotInstant& inst) {
                printf("RobotInstant[(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f), %.3f]\n",
                       inst.pose.position().x(), inst.pose.position().y(), inst.pose.heading(),
                       inst.velocity.linear().x(), inst.velocity.linear().y(), inst.velocity.angular(),
                       RJ::Seconds(inst.stamp-combo.begin_time()).count());
    };

    Trajectory partialPre = combo.subTrajectory(0s, 1.5s);
    Trajectory partialPost = RRTTrajectory(partialPre.last(), end, mot, obs);
    PlanAngles(partialPost, partialPre.last(), angleFn, rot);
    Trajectory combo2{partialPre, partialPost};


    for(int i = 0; i < combo2.num_instants(); i++) {
        printRobotInstant(combo2.instant(i));
    }
    cout << endl;

    for(int i = 0; i < combo.num_instants()-2; i++) {
        RobotInstant& i1 = combo.instant(i);
        RobotInstant& i2 = combo.instant(i+1);
        RobotInstant& i3 = combo.instant(i+2);

        double delta12 = fixAngleRadians(i2.pose.heading() - i1.pose.heading());
        double delta23 = fixAngleRadians(i3.pose.heading() - i2.pose.heading());

//        printRobotInstant(i1);
//        printRobotInstant(i2);
//        printRobotInstant(i3);
//        cout << endl;

        if(delta23 > .001) EXPECT_GT(delta12, .001);
        if(delta23 < -.001) EXPECT_LT(delta12, -.001);
        EXPECT_LT(i1.pose.heading(), i2.pose.heading());
        EXPECT_LT(i2.pose.heading(), i3.pose.heading());
        EXPECT_GE(i1.velocity.angular(), 0);
        EXPECT_GE(i2.velocity.angular(), 0);
    }
}