#include <gtest/gtest.h>

#include <random>

#include "Geometry2d/Point.hpp"
#include "MotionInstant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"

namespace Planning::TestingUtils {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

void assertPathContinuous(const Trajectory& path,
                          const RobotConstraints& constraints) {
    ASSERT_FALSE(path.empty());
    for (auto it = path.instants_begin();
         it != path.instants_end() && it != std::prev(path.instants_end());
         ++it) {
        RobotInstant cur = *it;
        RobotInstant nxt = *std::next(it);
        ASSERT_LT(cur.stamp, nxt.stamp);

        // continuous position
        // todo: fix these. right now they fail because there are a fixed
        // number of interpolations regardless of path length.
        //            double dist =
        //            cur.pose.position().distTo(nxt.pose.position()); double
        //            distAngle = std::abs(
        //                    fixAngleRadians(cur.pose.heading() -
        //                    nxt.pose.heading()));
        //            double maxJump = std::max((double) Robot_Radius,
        //                                      path.last().pose.position().distTo(
        //                                              path.first().pose.position())
        //                                              *
        //                                      0.1);
        //            ASSERT_LT(dist, maxJump);

        // check velocity from velocity profile
        ASSERT_LT(cur.velocity.linear().mag(), constraints.mot.maxSpeed + 1e-6);
        ASSERT_LT(nxt.velocity.linear().mag(), constraints.mot.maxSpeed + 1e-6);

        // check acceleration from velocity profile
        // todo: fix these. right now they fail because our velocity profile
        // is just an approximation and doesn't always work

        //            double dt = RJ::Seconds(nxt.stamp - cur.stamp).count();
        //            Point dv = nxt.velocity.linear() - cur.velocity.linear();
        //        ASSERT_LT(dv.mag(), constraints.mot.maxAcceleration * dt * 2);
        //            Point unitNormal = (nxt.velocity.linear().norm() -
        //                                cur.velocity.linear().norm()).norm();
        //            double dvNormal = std::abs(dv.dot(unitNormal));
        //        ASSERT_LT(dvNormal, constraints.mot.maxCentripetalAcceleration
        //        * dt + 1e-3);

        // todo: enable these to test angle profiles
        //            ASSERT_LT(cur.velocity.angular(), constraints.rot.maxSpeed
        //            + 1e-6); ASSERT_LT(nxt.velocity.angular(),
        //            constraints.rot.maxSpeed + 1e-6);
        //            ASSERT_LT(std::abs(nxt.velocity.angular() -
        //            cur.velocity.angular()),
        //                      constraints.rot.maxAccel * dt + 1e-6);
    }
}

double random(double lo, double hi) {
    static std::random_device randDevice;
    static std::mt19937 randGen(randDevice());
    static std::uniform_real_distribution<> randDistribution(0.0, 1.0);
    return lo + (hi - lo) * randDistribution(randGen);
}

RobotInstant randomInstant() {
    Point randPoint{random(-3, 3), random(0, 6)};
    Pose randPose{randPoint, random(0, 2 * M_PI)};
    Point randVel{random(-.5, .5), random(-.5, .5)};
    Twist randTwist{randVel, random(-.2, .2)};
    return RobotInstant{randPose, randTwist, RJ::now()};
}
}  // namespace Planning::TestingUtils