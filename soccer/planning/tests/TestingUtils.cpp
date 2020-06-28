#include <gtest/gtest.h>

#include <random>

#include "Geometry2d/Point.hpp"
#include "planning/Instant.hpp"
#include "planning/RobotConstraints.hpp"
#include "planning/Trajectory.hpp"

namespace Planning::TestingUtils {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

bool checkTrajectoryContinuous(const Trajectory& trajectory,
                               const RobotConstraints& constraints) {
    if (trajectory.empty()) {
        return false;
    }

    auto it = trajectory.instants_begin();
    RobotInstant previous = *it;
    it++;
    while (it != trajectory.instants_end()) {
        RobotInstant current = *it++;
        double dt = RJ::Seconds(current.stamp - previous.stamp).count();

        // Known issue: for very small delta-times, estimated acceleration (from
        // change in velocity) can be arbitrarily high.
        // TODO(PR #1492): Resolve this problem, and create velocity profiling
        // tests.
        if (dt < 1e-4) {
            previous = current;
            continue;
        }

        // Check for continuous position
        double dist = current.position().distTo(previous.position());
        double delta_angle =
            std::abs(fixAngleRadians(current.heading() - previous.heading()));
        double tangential_acceleration = (current.linear_velocity().mag() -
                                          previous.linear_velocity().mag()) /
                                         dt;

        // Include a 1.5x buffer on speeds, because our constraint logic isn't
        // perfect and this is just the average speed over the interval.
        // TODO(#1506): Check angle planning too.
        if (dist / dt > 1.5 * constraints.mot.maxSpeed) {
            std::cout << "Failure because of position deltas " << dist / dt
                      << ", " << delta_angle / dt << std::endl;
            return false;
        }

        // Make sure tangential acceleration and velocity are limited.
        if (current.linear_velocity().mag() > constraints.mot.maxSpeed * 1.5 ||
            tangential_acceleration > constraints.mot.maxAcceleration * 1.5) {
            std::cout << "Failure because of velocity "
                      << current.linear_velocity().mag() << ", acceleration "
                      << tangential_acceleration << " (max "
                      << constraints.mot.maxSpeed << ", "
                      << constraints.mot.maxAcceleration << ")"
                      << " dt = " << dt << " delta pos = " << dist << std::endl;
            return false;
        }

        previous = current;
    }

    return true;
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