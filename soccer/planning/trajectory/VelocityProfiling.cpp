#include "VelocityProfiling.hpp"
#include <motion/TrapezoidalMotion.hpp>
#include <Utils.hpp>

namespace Planning {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

Trajectory ProfileVelocity(const BezierPath& path,
                           double initial_speed,
                           double final_speed,
                           const MotionConstraints& constraints) {
    constexpr int num_segments = 15;

    // Interpolate Through Bezier Path
    std::vector<Point> points(num_segments + 1), derivs1(num_segments + 1);
    std::vector<double> curvature(num_segments + 1), speed(num_segments + 1);

    // Velocity pass: fill points and calculate maximum velocity given curvature
    // at each point.
    for (int n = 0; n < num_segments + 1; n++) {
        double s = n / static_cast<double>(num_segments);
        path.Evaluate(s, &points[n], &derivs1[n], &curvature[n]);
        speed[n] = constraints.maxSpeed;

        // Centripetal acceleration: a = v^2 / r => v = sqrt(ra)
        if (std::abs(curvature[n]) > 0) {
            speed[n] = std::max(speed[n], std::sqrt(constraints.maxAcceleration / curvature[n]));
        }
    }

    speed[0] = initial_speed;
    speed[num_segments] = final_speed;

    // Acceleration pass: calculate maximum velocity at each point based on
    // acceleration limits forwards in time.
    for (int n = 0; n < num_segments; n++) {
        double centripetal = speed[n] * speed[n] * curvature[n];

        using std::pow;
        double accel = std::sqrt(pow(constraints.maxAcceleration, 2) - pow(centripetal, 2));

        double distance = (points[n + 1] - points[n]).mag();

        // Do a trapezoid profile on maximum acceleration.
        // vf^2 = vi^2 + 2ad
        double max_speed = std::sqrt(pow(speed[n], 2) + 2 * accel * distance);
        if (max_speed < speed[n + 1]) {
            speed[n + 1] = max_speed;
        }
    }

    // Decceleration pass: calculate maximum velocity at each point based on
    // acceleration limits backwards in time.
    for (int n = num_segments; n > 0; n--) {
        double centripetal = speed[n] * speed[n] * curvature[n];

        using std::pow;
        double accel = std::sqrt(pow(constraints.maxAcceleration, 2) - pow(centripetal, 2));

        double distance = (points[n - 1] - points[n]).mag();

        // Do a trapezoid profile on maximum acceleration.
        // vf^2 = vi^2 + 2ad
        double max_speed = std::sqrt(pow(speed[n], 2) + 2 * accel * distance);
        if (max_speed < speed[n - 1]) {
            speed[n - 1] = max_speed;
        }
    }

    std::vector<RobotInstant> instants;

    // TODO(Kyle): Allow the user to pass in an initial time.
    RJ::Time time = RJ::now();

    instants.emplace_back(
            Pose(points[0], 0),
            Twist(derivs1[0].normalized() * speed[0], 0),
            time);
    for (int n = 1; n < num_segments + 1; n++) {
        // Add point n in
        instants.emplace_back(Pose(points[n], n), Twist(derivs1[n].normalized() * speed[n], n), time);

        double distance = (points[n] - points[n - 1]).mag();
        double vbar = (speed[n] + speed[n - 1]) / 2;
        double t_sec = distance / vbar;
        time = time + RJ::Seconds(t_sec);
    }

    return Trajectory(std::move(instants));
}

void PlanAngles(Trajectory& trajectory,
                RobotState initial_state,
                const AngleFunction& angle_function,
                const RotationConstraints& constraints) {
    trajectory.instant(0).pose = initial_state.pose;
    trajectory.instant(0).velocity = initial_state.velocity;

    // Move forwards in time. At each instant, calculate the goal angle and its
    // time derivative, and try to get there with a trapezoidal profile.
    for (int i = 0; i < trajectory.num_instants() - 1; i++) {
        RobotInstant& instant_initial = trajectory.instant(i);
        RobotInstant& instant_final = trajectory.instant(i + 1);

        double target_angle = angle_function(instant_final.pose.position(),
                                             instant_final.velocity.linear(),
                                             instant_initial.pose.heading());

        double target_angle_delta = fixAngleRadians(target_angle - instant_initial.pose.heading());
        double angle_delta;

        // TODO(Kyle): Calculate a proper final speed instead of specifying zero.
        TrapezoidalMotion(
                target_angle_delta,
                constraints.maxSpeed,
                constraints.maxAccel,
                std::chrono::duration_cast<RJ::Seconds>(instant_final.stamp - instant_initial.stamp).count(),
                instant_initial.velocity.angular(),
                0, angle_delta, instant_final.velocity.angular());
        instant_final.pose.heading() = instant_initial.pose.heading() + angle_delta;
    }
}

} // namespace Planning
