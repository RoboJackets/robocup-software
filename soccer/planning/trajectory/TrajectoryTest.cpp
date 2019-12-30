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

void assertEqual(const RobotInstant& inst1, const RobotInstant& inst2) {
    assert(inst1 == inst2);
    ASSERT_EQ(inst1, inst2);
}

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

    assertEqual(*trajectory.evaluate(start), start_instant);
    assertEqual(*trajectory.evaluate(trajectory.end_time()), mid_instant);
    EXPECT_FALSE(trajectory.evaluate(trajectory.begin_time() - RJ::Seconds(1e-3s)));
    EXPECT_FALSE(trajectory.evaluate(trajectory.end_time() + RJ::Seconds(1e-3s)));

    EXPECT_FALSE(Trajectory{{}}.evaluate(RJ::Seconds(0s)));
    EXPECT_FALSE(Trajectory{{}}.evaluate(RJ::now()));

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

//    Trajectory trajectory_2({});
//    trajectory_2.InsertInstant(start_instant);
//    trajectory_2.InsertInstant(end_instant);
//    trajectory_2.InsertInstant(mid_instant);
//
//    EXPECT_EQ(trajectory.duration(), trajectory_2.duration());
//    EXPECT_EQ(trajectory.instant(1), trajectory_2.instant(1));
}

TEST(Trajectory, TrajectoryIterator) {
    std::list<RobotInstant> instants;
    RobotInstant a{Pose{{1,1},1}, Twist{}, RJ::Time(1s)};
    RobotInstant b{Pose{{2,2}, 2}, Twist{}, RJ::Time(2s)};
    RobotInstant c{Pose{{5,5}, 5}, Twist{}, RJ::Time(5s)};
    instants.push_back(a);
    instants.push_back(b);
    instants.push_back(c);
    Trajectory traj{std::move(instants)};
    auto instants_it = traj.instants_begin();
    assertEqual(*instants_it, a);
    ++instants_it;
    ++instants_it;
    assertEqual(*instants_it, c);
    ++instants_it;
    ASSERT_EQ(instants_it, traj.instants_end());

    auto traj_it = traj.iterator(RJ::Time(1s), 0.5s);
    ASSERT_TRUE(traj_it.hasValue());
    ASSERT_TRUE(traj_it.hasNext());
    assertEqual(*traj_it, traj.first());
    for(int i = 0; i < 7; i++) ++traj_it;
    ASSERT_TRUE(traj_it.hasValue());
    ASSERT_TRUE(traj_it.hasNext());
    ++traj_it;
    assertEqual(*traj_it, traj.last());
    ASSERT_TRUE(traj_it.hasValue());
    ASSERT_FALSE(traj_it.hasNext());
    ++traj_it;
    ASSERT_FALSE(traj_it.hasValue());
    ASSERT_FALSE(traj_it.hasNext());
    Trajectory soloTraj{{RobotInstant{{}, {}, RJ::Time(10s)}}};
    auto itAfter = soloTraj.iterator(RJ::Time(20s), .5s);
    ASSERT_FALSE(itAfter.hasValue());
    ASSERT_FALSE(itAfter.hasNext());
    auto itBefore = soloTraj.iterator(RJ::Time(0s), .5s);
    ASSERT_FALSE(itAfter.hasValue());
    ASSERT_FALSE(itAfter.hasNext());
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
TEST(VelocityProfiling, Angular) {
    using namespace Geometry2d;
    using namespace Planning;
    using namespace std;

    MotionConstraints mot;
    RotationConstraints rot;

    /*
     * Test RRTTrajectory(), combo trajectory
     */
    RobotInstant start{Pose{{}, .1}, Twist{}, RJ::now()};
    RobotInstant mid{Pose{Point{1,1}, M_PI/2}, Twist{Point{0,1}, 0}, RJ::now()};
    RobotInstant end{Pose{Point{2,2}, M_PI/2}, Twist{Point{1,0}, 0}, RJ::now()};

    ShapeSet obs;
    Trajectory preTraj = RRTTrajectory(start, mid, mot, obs);
    ASSERT_FALSE(preTraj.empty());
    RJ::Time t0 = preTraj.begin_time();
    AngleFunction angleFn = [t0](const RobotInstant &instant) -> double {
        return RJ::Seconds(instant.stamp - t0).count();
    };
    PlanAngles(preTraj, start, angleFn, rot);
    Trajectory postTraj = RRTTrajectory(preTraj.last(), end, mot, obs);
    ASSERT_FALSE(postTraj.empty());
    PlanAngles(postTraj, preTraj.last(), angleFn, rot);
    Trajectory combo{preTraj, postTraj};

    for (auto it = preTraj.instants_begin();
         it != preTraj.instants_end(); ++it) {
        RobotInstant inst = *it;
//        printf("RobotInstant[(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f), %.3f]\n",
//               inst.pose.position().x(), inst.pose.position().y(), inst.pose.heading(),
//               inst.velocity.linear().x(), inst.velocity.linear().y(), inst.velocity.angular(),
//               RJ::Seconds(inst.stamp-preTraj.begin_time()).count());
        if (it != preTraj.instants_begin())
            EXPECT_NEAR(inst.pose.heading(), angleFn(inst), 1e-3);
    }
    cout << endl;
    for (auto it = postTraj.instants_begin();
         it != postTraj.instants_end(); ++it) {
        RobotInstant inst = *it;
//        printf("RobotInstant[(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f), %.3f]\n",
//               inst.pose.position().x(), inst.pose.position().y(), inst.pose.heading(),
//               inst.velocity.linear().x(), inst.velocity.linear().y(), inst.velocity.angular(),
//               RJ::Seconds(inst.stamp-postTraj.begin_time()).count());
        if (it != postTraj.instants_begin())
            EXPECT_NEAR(inst.pose.heading(), angleFn(inst), 1e-3);

    }
    cout << endl;
    for (auto it = combo.instants_begin();
         it != combo.instants_end(); ++it) {
        RobotInstant inst = *it;
//        printf("RobotInstant[(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f), %.3f]\n",
//               inst.pose.position().x(), inst.pose.position().y(), inst.pose.heading(),
//               inst.velocity.linear().x(), inst.velocity.linear().y(), inst.velocity.angular(),
//               RJ::Seconds(inst.stamp-combo.begin_time()).count());
        if (it != combo.instants_begin())
            EXPECT_NEAR(inst.pose.heading(), angleFn(inst), 1e-3);

    }
    cout << endl;

    /*
     * Test Partial Paths, Sub Trajectories
     */
    RJ::Seconds duration = combo.duration();
    Trajectory partialPre = combo.subTrajectory(0s, 1.5s);
    Trajectory partialPost = RRTTrajectory(partialPre.last(), end, mot,
                                           obs);
    PlanAngles(partialPost, partialPre.last(), angleFn, rot);
    Trajectory combo2{partialPre, partialPost};
    for (auto it = combo2.instants_begin();
         it != combo2.instants_end(); ++it) {
        RobotInstant inst = *it;
        //        printf("RobotInstant[(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f), %.3f]\n",
        //               inst.pose.position().x(), inst.pose.position().y(), inst.pose.heading(),
        //               inst.velocity.linear().x(), inst.velocity.linear().y(), inst.velocity.angular(),
        //               RJ::Seconds(inst.stamp-combo2.begin_time()).count());
        if (it != combo2.instants_begin())
            EXPECT_NEAR(inst.pose.heading(), angleFn(inst), 1e-3);

    }
    cout << endl;
}

TEST(Trajectory, Efficiency) {
    using namespace Geometry2d;
    using namespace Planning;
    using namespace std;
    /*
     * Test Bezier, VelocityProfile efficiency
     */
    MotionConstraints mot;
    RJ::Time effTestStartTime = RJ::now();
    BezierPath bezier({{0,0}, {1,1}},
                      {1,0},
                      {0,1},
                      mot);
    Trajectory traj = ProfileVelocity(bezier, 0, 0, mot);
    printf("time for Bezier, Profile: %.6f\n", RJ::Seconds(RJ::now()-effTestStartTime).count());

    RJ::Time t0 = RJ::now();
    ShapeSet obs;
    RRTTrajectory(RobotInstant{{{0,0},0}, {}, RJ::now()},
                  RobotInstant{{{1,1},0},{},RJ::now()},
                  mot, obs);
    printf("time for RRTTrajectory direct: %.6f\n", RJ::Seconds(RJ::now()-t0).count());

    t0 = RJ::now();
    obs.add(std::make_shared<Circle>(Point{.5,.5}, 0.2));
    RRTTrajectory(RobotInstant{{{0,0},0}, {}, RJ::now()},
            RobotInstant{{{1,1},0},{},RJ::now()},
            mot, obs);
    printf("time for RRTTrajectory obstructed: %.6f\n", RJ::Seconds(RJ::now()-t0).count());
}