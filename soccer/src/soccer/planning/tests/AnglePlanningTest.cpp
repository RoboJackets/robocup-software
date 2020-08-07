#include <fstream>

#include <gtest/gtest.h>

#include "planning/primitives/AnglePlanning.hpp"

using namespace Planning;
using namespace Geometry2d;

// (For the case where constraints are not saturated) check that we follow the
// angle function nearly exactly.
void check_angle_planning_near_exact(const Trajectory& trajectory,
                                     const AngleFunction& angle_function, double epsilon = 1e-2) {
    for (auto cursor = trajectory.cursor_begin(); cursor.has_value();
         cursor.advance(RJ::Seconds(0.01))) {
        RobotInstant instant = cursor.value();
        EXPECT_NEAR(instant.heading(),
                    angle_function(instant.linear_motion(), instant.heading(), nullptr), epsilon);
    }
}

static Trajectory make_straight_line_trajectory() {
    // Create a constant-velocity trajectory
    Trajectory trajectory;
    trajectory.AppendInstant(RobotInstant{Pose(0, 0, 2), Twist(1, 0, 0), RJ::Time(0s)});
    trajectory.AppendInstant(RobotInstant{Pose(0.5, 0, 0), Twist(1, 0, 0), RJ::Time(500ms)});
    trajectory.AppendInstant(RobotInstant{Pose(1, 0, 0), Twist(1, 0, 0), RJ::Time(1000ms)});
    trajectory.AppendInstant(RobotInstant{Pose(1.5, 0, 0), Twist(1, 0, 0), RJ::Time(1500ms)});
    trajectory.AppendInstant(RobotInstant{Pose(2, 0, 0), Twist(1, 0, 0), RJ::Time(2000ms)});
    trajectory.AppendInstant(RobotInstant{Pose(2.5, 0, 0), Twist(1, 0, 0), RJ::Time(2500ms)});
    trajectory.AppendInstant(RobotInstant{Pose(3, 0, 0), Twist(1, 0, 0), RJ::Time(3000ms)});
    return trajectory;
}

static RotationConstraints make_rotation_constraints() {
    RotationConstraints constraints;
    constraints.maxAccel = 1.0;
    constraints.maxSpeed = 3.0;
    return constraints;
}

// Make a RobotInstant with the given linear motion and exactly coinciding with
// the correct angle function.
static RobotInstant make_initial_instant(const LinearMotionInstant& linear, const RJ::Time& time,
                                         const AngleFunction& angle_fn, double approx_angle = 0.0) {
    Eigen::Vector2d gradient;
    double angle = angle_fn(linear, approx_angle, &gradient);
    double angular_velocity = gradient.dot(Eigen::Vector2d(linear.velocity));
    return RobotInstant{Pose{linear.position, angle}, Twist{linear.velocity, angular_velocity},
                        time};
}

// Dump the angle trajectory to a file. For debugging purposes.
[[maybe_unused]] static void dump_trajectory(const std::string& file_name,
                                             const Trajectory& trajectory,
                                             const AngleFunction& angle_fn) {
    std::ofstream out(file_name);
    auto cursor = trajectory.cursor(trajectory.begin_time());
    for (; cursor.has_value(); cursor.advance(0.01s)) {
        RobotInstant instant = cursor.value();

        double target = angle_fn(instant.linear_motion(), instant.heading(), nullptr);

        out << RJ::Seconds(instant.stamp - trajectory.begin_time()).count() << ", "
            << instant.heading() << ", " << instant.angular_velocity() << ", " << target
            << std::endl;
    }
}

TEST(AnglePlanning, obeys_angle_function_target) {
    auto trajectory = make_straight_line_trajectory();
    auto constraints = make_rotation_constraints();

    auto angle_fn = AngleFns::facePoint(Point(2, 0.5));

    // Create a robot instant. This should allow us to have perfect tracking,
    // because we will start perfectly in line with the angle function.
    RobotInstant initial_instant = make_initial_instant(trajectory.first().linear_motion(),
                                                        trajectory.first().stamp, angle_fn);

    PlanAngles(&trajectory, initial_instant, angle_fn, constraints);

    check_angle_planning_near_exact(trajectory, angle_fn, 5e-2);
}

TEST(AnglePlanning, DISABLED_start_from_incorrect_angle) {
    auto trajectory = make_straight_line_trajectory();
    auto constraints = make_rotation_constraints();

    auto angle_fn = AngleFns::facePoint(Point(2, 0.5));

    // This time we will use a robot instant with zero heading/angular velocity,
    // which doesn't match up with the profile. This means that we need to catch
    // up.
    RobotInstant initial_instant = trajectory.first();

    PlanAngles(&trajectory, initial_instant, angle_fn, constraints);

    // Check that we start in the right place.
    EXPECT_TRUE(RobotInstant::nearly_equals(trajectory.first(), initial_instant));

    // Check that by the latter half of the trajectory, we are close to correct
    check_angle_planning_near_exact(
        trajectory.subTrajectory(trajectory.begin_time() + RJ::Seconds(2.0), trajectory.end_time()),
        angle_fn, 5e-2);
}
