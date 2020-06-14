#include <gtest/gtest.h>

#include "planning/low_level/RRTUtil.hpp"

#if 0
TEST(Trajectory, Efficiency) {
    using namespace Geometry2d;
    using namespace Planning;
    using namespace std;
    /*
     * Test Bezier, VelocityProfile efficiency
     */
    MotionConstraints mot;
    RJ::Time effTestStartTime = RJ::now();
    BezierPath bezier({{0, 0}, {1, 1}}, {1, 0}, {0, 1}, mot);
    Trajectory traj = ProfileVelocity(bezier, 0, 0, mot);
    printf("time for Bezier, Profile: %.6f\n",
           RJ::Seconds(RJ::now() - effTestStartTime).count());

    ShapeSet obs;

    RJ::Time t0 = RJ::now();
    auto state_space = std::make_shared<RoboCupStateSpace>(
        Field_Dimensions::Current_Dimensions, obs);
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(Point{0, 0});
    biRRT.setGoalState(Point{1, 1});
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
    CreatePath::rrt(LinearMotionInstant{Point(0, 0)},
                    LinearMotionInstant{Point(1, 1)},
                    mot, RJ::now(), obs);
    printf("time for PathGen::rrt direct: %.6f\n",
           RJ::Seconds(RJ::now() - t0).count());

    t0 = RJ::now();
    obs.add(std::make_shared<Circle>(Point{.5, .5}, 0.2));
    CreatePath::rrt(LinearMotionInstant{Point()},
                    LinearMotionInstant{Point(1, 1)},
                    mot, RJ::now(), obs);
    printf("time for PathGen::rrt obstructed: %.6f\n",
           RJ::Seconds(RJ::now() - t0).count());
}

TEST(RRT, Time) {
    using namespace std;
    using namespace Planning;
    using namespace Geometry2d;
    RJ::Time t0 = RJ::now();
    ShapeSet obs;
    auto state_space = std::make_shared<RoboCupStateSpace>(
        Field_Dimensions::Current_Dimensions, obs);
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(Point{0, 0});
    biRRT.setGoalState(Point{1, 1});
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

TEST(Trajectory, createRRTSmall) {
    RJ::Time time = RJ::now();
    RobotInstant start{Pose{{}, .1}, Twist{}, time};
    Trajectory a = CreatePath::rrt(start.linear_motion(),
                                   start.linear_motion(),
                                   MotionConstraints{},
                                   start.stamp, {});
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
    RobotInstant mid{Pose{Point{1, 1}, 0}, Twist{Point{0, .1}, 0}, RJ::now()};
    RobotInstant end{Pose{Point{2, 2}, 0}, Twist{Point{.1, 0}, 0}, RJ::now()};
    ShapeSet obs;
    RJ::Time t0 = start.stamp;
    AngleFunction angleFn = [t0](const RobotInstant& instant) -> double {
        return RJ::Seconds(instant.stamp - t0).count();
    };

    /*
     * Test PathGen::rrt(), combo trajectory
     */
    Trajectory preTraj = CreatePath::rrt(start.linear_motion(),
                                         mid.linear_motion(),
                                         constraints.mot,
                                         start.stamp, obs);
    PlanAngles(&preTraj,
               start,
               angleFn,
               constraints.rot);
    assertPathContinuous(preTraj, constraints);

    Trajectory postTraj =
        CreatePath::rrt(preTraj.last().linear_motion(),
                        end.linear_motion(), constraints.mot,
                        preTraj.end_time(), obs);

    PlanAngles(&postTraj,
               preTraj.last(),
               angleFn,
               constraints.rot);
    assertPathContinuous(postTraj, constraints);
    Trajectory combo{preTraj, postTraj};
    assertPathContinuous(combo, RobotConstraints{});

    /*
     * Test Partial Paths, Sub Trajectories
     */
    Trajectory partialPre = combo.subTrajectory(0s, 1.5s);
    assertPathContinuous(partialPre, constraints);
    Trajectory partialPost =
        CreatePath::rrt(partialPre.last().linear_motion(),
                        end.linear_motion(), constraints.mot,
                        partialPre.end_time(), obs);
    PlanAngles(&partialPost,
               partialPre.last(),
               angleFn,
               constraints.rot);
    assertPathContinuous(partialPost, constraints);
    Trajectory combo2{partialPre, partialPost};
    assertPathContinuous(combo2, constraints);
}

TEST(Trajectory, CreateRRTSuccessRate) {
    int fails = 0;
    constexpr int iterations = 1000;
    constexpr int numTries = 300;
    for (int i = 0; i < iterations; i++) {
        ShapeSet obstacles;
        int numObstacles = (int)random(2, 5);
        for (int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{random(-2, 2), random(2, 3)}, .2));
        }
        RobotInstant start{
            Pose{Point{random(-3, 3), random(5, 5.5)}, random(0, 2 * M_PI)},
            Twist{Point{random(-.5, .5), random(-.5, .5)}, 0}, RJ::now()};
        RobotInstant goal{
            Pose{Point{random(-3, 3), random(0.5, 1)}, random(0, 2 * M_PI)},
            Twist{Point{random(-.5, .5), random(-.5, .5)}, 0}, RJ::now()};
        Trajectory path{{}};
        for (int j = 0; j < numTries && path.empty(); j++) {
            path = CreatePath::rrt(start.linear_motion(), goal.linear_motion(),
                                   MotionConstraints{}, start.stamp, obstacles);
            if (path.empty()) {
                fails++;
            }
            ASSERT_TRUE(j != numTries - 1);
        }
        assertPathContinuous(path, RobotConstraints{});
    }
    double successRate = (double)(iterations) / (iterations + fails);
    printf("PathGen::rrt() Success Rate: %.6f\n", successRate);
}

TEST(Trajectory, AngleProfileNoExtraInstants) {
    RobotInstant start{Pose{{}, 0}, {}, RJ::now()};
    RobotInstant goal{Pose{{3, 0}, 0}, {}, RJ::now()};
    Trajectory path = CreatePath::rrt(start.linear_motion(),
                                      goal.linear_motion(),
                                      MotionConstraints{},
                                      start.stamp, {}, {});
    RotationConstraints rot;
    double maxDeltaAngle =
        rot.maxAccel * std::pow(path.duration().count() / 2.0, 2);
    PlanAngles(&path,
               start,
               AngleFns::faceAngle(maxDeltaAngle * 0.5),
               rot);
    assertPathContinuous(path, RobotConstraints{});
}

void assertPivotEndpoints(double a0, double af, double w0) {
    RobotConstraints constraints;
    RobotInstant start{Pose{{}, a0}, Twist{{}, w0}, RJ::now()};
    RobotInstant goal{Pose{{}, af}, Twist{{}, 0}, RJ::now()};
    Trajectory out = CreatePath::rrt(start.linear_motion(),
                                     goal.linear_motion(),
                                     constraints.mot, start.stamp, {});
    ASSERT_FALSE(out.empty());
    PlanAngles(&out,
               start,
               AngleFns::faceAngle(af),
               constraints.rot);
    // check first instant
    ASSERT_NEAR(out.first().position().distTo(start.position()), 0,
                1e-6);
    ASSERT_NEAR(fixAngleRadians(out.first().pose.heading()),
                fixAngleRadians(a0), 1e-6);
    ASSERT_NEAR(out.first().velocity.angular(), w0, 1e-6);
    ASSERT_NEAR(out.first().velocity.linear().mag(), 0, 1e-6);
    // check last instant
    ASSERT_NEAR(out.last().position().distTo(start.position()), 0,
                1e-6);
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
    Trajectory out = CreatePath::rrt(start.linear_motion(),
                                     goal.linear_motion(),
                                     constraints.mot, start.stamp, {});
    ASSERT_FALSE(out.empty());

    PlanAngles(&out,
               start,
               AngleFns::faceAngle(af),
               constraints.rot);

    // check first instant
    ASSERT_NEAR(out.first().position().distTo(start.position()), 0,
                1e-6);
    ASSERT_NEAR(fixAngleRadians(out.first().pose.heading()),
                fixAngleRadians(a0), 1e-6);
    ASSERT_NEAR(out.first().velocity.angular(), w0, 1e-6);
    ASSERT_NEAR(out.first().velocity.linear().mag(), 0, 1e-6);
    // check last instant
    ASSERT_NEAR(out.last().position().distTo(start.position()), 0,
                1e-6);
    ASSERT_NEAR(fixAngleRadians(out.last().pose.heading()), fixAngleRadians(af),
                1e-6);
    ASSERT_NEAR(out.last().velocity.linear().mag(), 0, 1e-6);
    ASSERT_NEAR(out.last().velocity.angular(), 0, 1e-6);
    // check middle instant
    double deltaAngle = fixAngleRadians(af - a0);
    RJ::Seconds halfwayTime(-99.0);
    std::optional<RobotInstant> evalHalfway;
    if (w0 * deltaAngle > 1e-6) {
        halfwayTime = out.duration() * 0.5;
    } else {
        double returnTime =
            std::sqrt(2 * std::abs(w0) / constraints.rot.maxAccel);
        std::optional<RobotInstant> evalReturn =
            out.evaluate(RJ::Seconds(returnTime));
        ASSERT_TRUE(evalReturn);
        ASSERT_NEAR(fixAngleRadians(evalReturn->pose.heading() - a0), 0, 1e-6);
        ASSERT_NEAR(evalReturn->velocity.angular(), -w0, 1e-6);
        halfwayTime =
            RJ::Seconds(out.duration() - RJ::Seconds(returnTime)) * 0.5;
    }
    evalHalfway = out.evaluate(halfwayTime);
    ASSERT_NEAR(RJ::Seconds(evalHalfway->stamp - out.begin_time()).count(),
                halfwayTime.count(), 1e-6);
    ASSERT_TRUE(evalHalfway);
    double accel = constraints.rot.maxAccel * (deltaAngle > 0 ? 1 : -1);
    double beginVel = std::abs(w0) * (deltaAngle > 0 ? 1 : -1);
    double halfwayVel = beginVel + accel * halfwayTime.count();
    double halfwayAngleDelta =
        (std::pow(halfwayVel, 2) - std::pow(beginVel, 2)) / (2 * accel);
    if (std::abs(halfwayVel) > constraints.rot.maxSpeed) {
        halfwayVel = constraints.rot.maxSpeed * (halfwayVel > 0 ? 1 : -1);
        double accelTime = (halfwayVel - beginVel) / accel;
        assert(accelTime > 0);
        double constantVelTime = halfwayTime.count() - accelTime;
        halfwayAngleDelta =
            (std::pow(halfwayVel, 2) - std::pow(beginVel, 2)) / (2 * accel) +
            halfwayVel * constantVelTime;
    }
    ASSERT_NEAR(fixAngleRadians(evalHalfway->pose.heading() - a0),
                halfwayAngleDelta, 1e-6);
    ASSERT_NEAR(evalHalfway->position().distTo(start.position()), 0,
                1e-6);
    ASSERT_NEAR(evalHalfway->velocity.linear().mag(), 0, 1e-6);
    ASSERT_NEAR(evalHalfway->velocity.angular(), halfwayVel, 1e-6);

    assertPathContinuous(out, constraints);
}
#endif
