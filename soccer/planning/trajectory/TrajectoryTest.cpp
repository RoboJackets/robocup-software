#include <gtest/gtest.h>
#include "Trajectory.hpp"
#include "PathSmoothing.hpp"
#include "planning/trajectory/RRTUtil.hpp"
#include "VelocityProfiling.hpp"
#include "planning/planner/PathTargetPlanner.hpp"
#include <rrt/planning/Path.hpp>
#include <random>

using namespace Planning;
using namespace Geometry2d;

void assertPathContinuous(const Trajectory& path, const RobotConstraints& constraints);

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

    ASSERT_EQ(*trajectory.evaluate(start), start_instant);
    ASSERT_EQ(*trajectory.evaluate(trajectory.end_time()), mid_instant);
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
    ASSERT_EQ(*instants_it, a);
    ++instants_it;
    ++instants_it;
    ASSERT_EQ(*instants_it, c);
    ++instants_it;
    ASSERT_EQ(instants_it, traj.instants_end());

    auto traj_it = traj.iterator(RJ::Time(1s), 0.5s);
    ASSERT_TRUE(traj_it.hasValue());
    ASSERT_TRUE(traj_it.hasNext());
    ASSERT_EQ(*traj_it, traj.first());
    for(int i = 0; i < 7; i++) ++traj_it;
    ASSERT_TRUE(traj_it.hasValue());
    ASSERT_TRUE(traj_it.hasNext());
    ++traj_it;
    ASSERT_EQ(*traj_it, traj.last());
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

TEST(Trajectory, BezierPath) {
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

    for(int i = 0; i < 4; i++) {
        Point p, v;
        double k;
        path.Evaluate(i / 3.0, &p, &v, &k);

        std::cout << p << ", " << v << std::endl;
        EXPECT_NEAR((p - points[i]).mag(), 0, 1e-6);
        if(i == 0) {
            EXPECT_NEAR(v.angleBetween(vi), 0, 1e-3);
        }
        if(i == 3) {
            EXPECT_NEAR(v.angleBetween(vf), 0, 1e-3);
        }
    }
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

    ShapeSet obs;

    RJ::Time t0 = RJ::now();
    auto state_space = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, obs);
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(Point{0,0});
    biRRT.setGoalState(Point{1,1});
    biRRT.setStepSize(0.15);
    biRRT.setMinIterations(100);
    biRRT.setMaxIterations(250);
    biRRT.setGoalBias(0.3);
    printf("ConfigTime: %.6f\n", RJ::Seconds(RJ::now() - t0).count());
    biRRT.run();
    vector<Point> points = biRRT.getPath();
    RRT::SmoothPath(points, *state_space);
    printf("RRTTime: %.6f\n", RJ::Seconds(RJ::now() - t0).count());


    t0 = RJ::now();
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

TEST(RRT, Time) {
    using namespace std;
    using namespace Planning;
    using namespace Geometry2d;
    RJ::Time t0 = RJ::now();
    ShapeSet obs;
    auto state_space = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, obs);
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(Point{0,0});
    biRRT.setGoalState(Point{1,1});
    biRRT.setStepSize(0.15);
    biRRT.setMinIterations(100);
    biRRT.setMaxIterations(250);
    biRRT.setGoalBias(0.3);
    printf("ConfigTime: %.6f\n", RJ::Seconds(RJ::now() - t0).count());
    biRRT.run();
    vector<Point> points = biRRT.getPath();
    RRT::SmoothPath(points, *state_space);
    printf("RRTTime: %.6f\n", RJ::Seconds(RJ::now() - t0).count());
}
namespace Testing {
double rando(double lo, double hi) {
    static std::random_device randDevice;
    static std::mt19937 randGen(randDevice());
    static std::uniform_real_distribution<> randDistribution(0.0, 1.0);
    return lo + (hi-lo) * randDistribution(randGen);
}
}
using namespace Testing;

TEST(Trajectory, RRTTrajectorySmall) {
    RobotInstant start{Pose{{}, .1}, Twist{}, RJ::now()};
    Trajectory a = RRTTrajectory(start, start, MotionConstraints{}, {});
    ASSERT_FALSE(a.empty());
    ASSERT_TRUE(a.num_instants() == 1);
    ASSERT_NEAR(a.duration().count(), 0.0, 1e-6);
}

namespace Planning {
double clampAccel(double v1, double v2, double deltaX, double maxAccel);
}
TEST(Trajectory, VelocityProfileClampAccel) {
    ASSERT_NEAR(clampAccel(1, 20, 1.5, 1), 2, 1e-6);
    ASSERT_NEAR(clampAccel(5, -20, 1.5, 3), 4, 1e-6);
    ASSERT_NEAR(clampAccel(-1, -20, -1.5, 1), -2, 1e-6);
    ASSERT_NEAR(clampAccel(-5, 20, -1.5, 3), -4, 1e-6);
    ASSERT_NEAR(clampAccel(0, 5, 0.5, 1), 1, 1e-6);
    ASSERT_NEAR(clampAccel(0, -5, -0.5, 1), -1, 1e-6);
    ASSERT_NEAR(clampAccel(1, -1, 0.5, 1), 0, 1e-6);
    ASSERT_NEAR(clampAccel(-1, 1, -0.5, 1), 0, 1e-6);
}
TEST(Trajectory, CombiningTrajectories_and_SubTrajectories) {
    RobotConstraints constraints;
    RobotInstant start{Pose{{}, .1}, Twist{}, RJ::now()};
    RobotInstant mid{Pose{Point{1,1}, 0}, Twist{Point{0,.1}, 0}, RJ::now()};
    RobotInstant end{Pose{Point{2,2}, 0}, Twist{Point{.1,0}, 0}, RJ::now()};
    ShapeSet obs;
    RJ::Time t0 = start.stamp;
    AngleFunction angleFn = [t0](const RobotInstant &instant) -> double {
        return RJ::Seconds(instant.stamp - t0).count();
    };

    /*
     * Test RRTTrajectory(), combo trajectory
     */
    Trajectory preTraj = RRTTrajectory(start, mid, constraints.mot, obs);
    PlanAngles(preTraj, start, angleFn, constraints.rot);
    assertPathContinuous(preTraj, constraints);
    Trajectory postTraj = RRTTrajectory(preTraj.last(), end, constraints.mot, obs);
    PlanAngles(postTraj, preTraj.last(), angleFn, constraints.rot);
    assertPathContinuous(postTraj, constraints);
    Trajectory combo{preTraj, postTraj};
    assertPathContinuous(combo, RobotConstraints{});

    /*
     * Test Partial Paths, Sub Trajectories
     */
    Trajectory partialPre = combo.subTrajectory(0s, 1.5s);
    assertPathContinuous(partialPre, constraints);
    Trajectory partialPost = RRTTrajectory(partialPre.last(), end, constraints.mot,
                                           obs);
    PlanAngles(partialPost, partialPre.last(), angleFn, constraints.rot);
    assertPathContinuous(partialPost, constraints);
    Trajectory combo2{partialPre, partialPost};
    assertPathContinuous(combo2, constraints);
}

TEST(Trajectory, RRTTrajectorySuccessRate) {
    int fails = 0;
    constexpr int iterations = 1000;
    constexpr int numTries = 300;
    for(int i = 0; i < iterations; i++) {
        ShapeSet obstacles;
        int numObstacles = (int)rando(2, 5);
        for(int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(Point{rando(-2, 2), rando(2, 3)}, .2));
        }
        RobotInstant start{Pose{Point{rando(-3, 3), rando(5, 5.5)}, rando(0, 2*M_PI)}, Twist{Point{rando(-.5,.5), rando(-.5,.5)}, 0}, RJ::now()};
        RobotInstant goal{Pose{Point{rando(-3, 3), rando(0.5, 1)}, rando(0, 2*M_PI)}, Twist{Point{rando(-.5,.5), rando(-.5,.5)}, 0}, RJ::now()};
        Trajectory path{{}};
        for (int j = 0; j < numTries && path.empty(); j++){
            path = RRTTrajectory(start, goal, MotionConstraints{}, obstacles);
            if(path.empty()) {
                fails++;
            }
            ASSERT_TRUE(j != numTries-1);
        }
        assertPathContinuous(path, RobotConstraints{});
    }
    double successRate = (double)(iterations) / (iterations + fails);
    printf("RRTTrajectory() Success Rate: %.6f\n", successRate);
}

void assertPivotEndpoints(double a0, double af, double w0) {
    RobotConstraints constraints;
    RobotInstant start{Pose{{}, a0}, Twist{{}, w0}, RJ::now()};
    RobotInstant goal{Pose{{}, af}, Twist{{}, 0}, RJ::now()};
    Trajectory out = RRTTrajectory(start, goal, constraints.mot, {});
    ASSERT_FALSE(out.empty());
    PlanAngles(out, start, AngleFns::faceAngle(af), constraints.rot);
    // check first instant
    ASSERT_NEAR(out.first().pose.position().distTo(start.pose.position()), 0, 1e-6);
    ASSERT_NEAR(fixAngleRadians(out.first().pose.heading()), fixAngleRadians(a0), 1e-6);
    ASSERT_NEAR(out.first().velocity.angular(), w0, 1e-6);
    ASSERT_NEAR(out.first().velocity.linear().mag(), 0, 1e-6);
    // check last instant
    ASSERT_NEAR(out.last().pose.position().distTo(start.pose.position()), 0, 1e-6);
    double fao = fixAngleRadians(a0);
    double faf = fixAngleRadians(af);
    double flast = fixAngleRadians(out.last().pose.heading());
    ASSERT_NEAR(faf, flast, 1e-6);
    ASSERT_NEAR(out.last().velocity.linear().mag(), 0, 1e-6);
    ASSERT_NEAR(out.last().velocity.angular(), 0, 1e-6);
}
void assertPivot(double a0, double af, double w0) {
    RobotConstraints constraints;
    RobotInstant start{Pose{{}, a0}, Twist{{}, w0}, RJ::now()};
    RobotInstant goal{Pose{{}, af}, Twist{{}, 0}, RJ::now()};
    Trajectory out = RRTTrajectory(start, goal, constraints.mot, {});
    ASSERT_FALSE(out.empty());
    PlanAngles(out, start, AngleFns::faceAngle(af), constraints.rot);
    // check first instant
    ASSERT_NEAR(out.first().pose.position().distTo(start.pose.position()), 0, 1e-6);
    ASSERT_NEAR(fixAngleRadians(out.first().pose.heading()), fixAngleRadians(a0), 1e-6);
    ASSERT_NEAR(out.first().velocity.angular(), w0, 1e-6);
    ASSERT_NEAR(out.first().velocity.linear().mag(), 0, 1e-6);
    // check last instant
    ASSERT_NEAR(out.last().pose.position().distTo(start.pose.position()), 0, 1e-6);
    ASSERT_NEAR(fixAngleRadians(out.last().pose.heading()), fixAngleRadians(af), 1e-6);
    ASSERT_NEAR(out.last().velocity.linear().mag(), 0, 1e-6);
    ASSERT_NEAR(out.last().velocity.angular(), 0, 1e-6);
    //check middle instant
    double deltaAngle = fixAngleRadians(af - a0);
    RJ::Seconds halfwayTime(-99.0);
    std::optional<RobotInstant> evalHalfway;
    if(w0 * deltaAngle > 1e-6) {
        halfwayTime = out.duration() * 0.5;
    } else {
        double returnTime = std::sqrt(2 * std::abs(w0) / constraints.rot.maxAccel);
        std::optional<RobotInstant> evalReturn = out.evaluate(RJ::Seconds(returnTime));
        ASSERT_TRUE(evalReturn);
        ASSERT_NEAR(fixAngleRadians(evalReturn->pose.heading()-a0), 0, 1e-6);
        ASSERT_NEAR(evalReturn->velocity.angular(), -w0, 1e-6);
        halfwayTime = RJ::Seconds(out.duration() - RJ::Seconds(returnTime)) * 0.5;
    }
    evalHalfway = out.evaluate(halfwayTime);
    ASSERT_NEAR(RJ::Seconds(evalHalfway->stamp - out.begin_time()).count(), halfwayTime.count(), 1e-6);
    ASSERT_TRUE(evalHalfway);
    double accel = constraints.rot.maxAccel * (deltaAngle > 0 ? 1 : -1);
    double beginVel = std::abs(w0) *( deltaAngle > 0 ? 1 : -1);
    double halfwayVel = beginVel + accel * halfwayTime.count();
    double halfwayAngleDelta = (std::pow(halfwayVel, 2) - std::pow(beginVel, 2)) / (2 * accel);
    if(std::abs(halfwayVel) > constraints.rot.maxSpeed) {
        halfwayVel = constraints.rot.maxSpeed * (halfwayVel > 0 ? 1 : -1);
        double accelTime = (halfwayVel - beginVel) / accel;
        assert(accelTime > 0);
        double constantVelTime = halfwayTime.count() - accelTime;
        halfwayAngleDelta = (std::pow(halfwayVel, 2) - std::pow(beginVel, 2)) / (2 * accel)
                            + halfwayVel * constantVelTime;
    }
    //todo fix this
    ASSERT_NEAR(fixAngleRadians(evalHalfway->pose.heading() - a0), halfwayAngleDelta, 1e-6);//todo(Ethan)
    ASSERT_NEAR(evalHalfway->pose.position().distTo(start.pose.position()), 0, 1e-6);
    ASSERT_NEAR(evalHalfway->velocity.linear().mag(), 0, 1e-6);
    ASSERT_NEAR(evalHalfway->velocity.angular(), halfwayVel, 1e-6);

    assertPathContinuous(out, constraints);
}

TEST(Trajectory, PivotTurnEndpointsOnly) {
    double maxSpeed = RotationConstraints{}.maxSpeed;
    for(int i = 0; i < 1000; i++) {
        assertPivotEndpoints(rando(-10*M_PI, 10*M_PI), rando(-10*M_PI, 10*M_PI), rando(-maxSpeed, maxSpeed));
    }
}
//todo(Ethan) should probably make this pass...
//TEST(Trajectory, PivotTurn) {
//    double maxSpeed = RotationConstraints{}.maxSpeed;
//    for(int i = 0; i < 1000; i++) {
//        assertPivot(rando(-10*M_PI, 10*M_PI), rando(-10*M_PI, 10*M_PI), rando(-maxSpeed, maxSpeed));
//    }
//}