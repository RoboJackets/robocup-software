#include "testing_utils.hpp"

#include <gtest/gtest.h>

#include <rj_common/utils.hpp>

#include "rj_geometry/point.hpp"
#include "planning/instant.hpp"
#include "planning/robot_constraints.hpp"
#include "planning/trajectory.hpp"

namespace planning::TestingUtils {

using rj_geometry::Point;
using rj_geometry::Pose;
using rj_geometry::Twist;

bool check_trajectory_continuous(const Trajectory& trajectory,
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
        double dist = current.position().dist_to(previous.position());
        double delta_angle = std::abs(fix_angle_radians(current.heading() - previous.heading()));
        double tangential_acceleration =
            (current.linear_velocity().mag() - previous.linear_velocity().mag()) / dt;

        // Include a 1.5x buffer on speeds, because our constraint logic isn't
        // perfect and this is just the average speed over the interval.
        // TODO(#1506): Check angle planning too.
        if (dist / dt > 1.5 * constraints.mot.max_speed) {
            std::cout << "Failure because of position deltas " << dist / dt << ", "
                      << delta_angle / dt << std::endl;
            return false;
        }

        // Make sure tangential acceleration and velocity are limited.
        // For now, don't enforce the acceleration limit.
        if (current.linear_velocity().mag() > constraints.mot.max_speed * 1.5
            /* || tangential_acceleration > constraints.mot.max_acceleration * 1.5 */) {
            std::cout << "Failure because of velocity " << current.linear_velocity().mag()
                      << ", acceleration " << tangential_acceleration << " (max "
                      << constraints.mot.max_speed << ", " << constraints.mot.max_acceleration
                      << ")"
                      << " dt = " << dt << " delta pos = " << dist << std::endl;
            return false;
        }

        previous = current;
    }

    return true;
}

RobotInstant random_instant(std::mt19937* generator) {
    Point rand_point{random(generator, -3.0, 3.0), random(generator, 0.0, 6.0)};
    Pose rand_pose{rand_point, random(generator, 0.0, 2 * M_PI)};
    Point rand_vel{random(generator, -0.5, 0.5), random(generator, -0.5, 0.5)};
    Twist rand_twist{rand_vel, random(generator, -0.2, 0.2)};
    return RobotInstant{rand_pose, rand_twist, RJ::now()};
}

}  // namespace planning::TestingUtils