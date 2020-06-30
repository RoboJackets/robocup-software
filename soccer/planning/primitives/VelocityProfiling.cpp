#include "VelocityProfiling.hpp"

#include "TrapezoidalMotion.hpp"
#include "planning/Instant.hpp"

namespace Planning {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

constexpr int kInterpolationsPerBezier = 40;

double limit_acceleration(double velocity_initial, double velocity_final,
                          double displacement, double max_accel) {
    double offset = std::abs(2 * max_accel * displacement);
    return std::clamp(velocity_final,
                      std::sqrt(std::pow(velocity_initial, 2) - offset),
                      std::sqrt(std::pow(velocity_initial, 2) + offset));
}

Trajectory ProfileVelocity(const BezierPath& path, double initial_speed,
                           double final_speed,
                           const MotionConstraints& constraints,
                           RJ::Time initial_time) {
    if (path.empty()) {
        return Trajectory{{}};
    }

    // Number of cubic bezier segments
    const int num_beziers = path.size();

    // Number of points that will be in the final trajectory. We need to add one
    // to account for the final instant
    const int num_points = num_beziers * kInterpolationsPerBezier + 1;
    const int num_segments = num_points - 1;

    // Scratch data that we will use later.
    std::vector<Point> points(num_points);
    std::vector<Point> derivs1(num_points);
    std::vector<double> curvature(num_points);
    std::vector<double> speed(num_points, constraints.maxSpeed);

    // Note: these are just suggestions. If they are impossible given
    // MotionConstraints, then we'll limit them.
    speed[0] = initial_speed;
    speed[num_points - 1] = std::min(constraints.maxSpeed, final_speed);

    // Disable curvature limiting. Our Bezier implementation has some
    // issues when the initial or final velocity is small: it places
    // two keypoints directly on top of one another, which makes very large
    // curvature near the endpoints.
    // TODO(#1539): Switch to Hermite splines and minimize
    //  sum-squared-acceleration instead of solving for Bezier curves.
    // double max_centripetal_acceleration = constraints.maxAcceleration;

    // Velocity pass: fill points and calculate maximum velocity given curvature
    // at each point.
    for (int n = 0; n < num_points; n++) {
        double s = n / static_cast<double>(num_segments);
        path.Evaluate(s, &points[n], &derivs1[n], &curvature[n]);

        if (curvature[n] < 0.0 || !std::isfinite(curvature[n])) {
            throw std::runtime_error("Invalid curvature");
        }

        // TODO(#1539): Re-enable curvature limiting
        // Centripetal acceleration: a = v^2 / r => v = sqrt(ra)
        // if (limit_curvature && curvature[n] != 0.0) {
        //     speed[n] = std::min(
        //         speed[n],
        //         std::sqrt(max_centripetal_acceleration / curvature[n]));
        // }
    }

    // Acceleration pass: calculate maximum velocity at each point based on
    // acceleration limits forwards in time.
    for (int n = 0; n < num_points - 2; n++) {
        double max_tangential_acceleration = constraints.maxAcceleration;

        // TODO(#1539): Re-enable curvature limiting
        // if (limit_curvature) {
        //     double centripetal_acceleartion =
        //         speed[n] * speed[n] * curvature[n];
        //     double squared_max_tangential_acceleration =
        //         std::pow(constraints.maxAcceleration, 2) -
        //         std::pow(centripetal_acceleartion, 2);

        //     // This can occur when our initial speed is fast enough that we
        //     // will slip no matter what.
        //     if (squared_max_tangential_acceleration <= 0) {
        //         squared_max_tangential_acceleration = 0;
        //     }

        //     max_tangential_acceleration =
        //         std::sqrt(squared_max_tangential_acceleration);
        // }

        if (!std::isfinite(max_tangential_acceleration)) {
            throw std::runtime_error("Invalid maximum tangential acceleration");
        }

        double distance = (points[n + 1] - points[n]).mag();
        speed[n + 1] = limit_acceleration(speed[n], speed[n + 1], distance,
                                          max_tangential_acceleration);
    }

    // Deceleration pass: calculate maximum velocity at each point based on
    // acceleration limits backwards in time.
    for (int n = num_points - 1; n > 1; n--) {
        double max_tangential_acceleration = constraints.maxAcceleration;

        // TODO(#1539): Re-enable curvature limiting
        // if (limit_curvature) {
        //     double centripetal_acceleration =
        //         speed[n] * speed[n] * curvature[n];
        //     double squared_max_tangential_acceleration =
        //         std::pow(constraints.maxAcceleration, 2) -
        //         std::pow(centripetal_acceleration, 2);

        //     // This can occur when our initial speed is fast enough that we
        //     // will slip no matter what.
        //     if (squared_max_tangential_acceleration <= 0) {
        //         squared_max_tangential_acceleration = 0;
        //     }

        //     double max_tangential_acceleration =
        //         std::sqrt(squared_max_tangential_acceleration);
        // }

        if (!std::isfinite(max_tangential_acceleration)) {
            throw std::runtime_error("Invalid max tangential acceleration");
        }

        double distance = (points[n - 1] - points[n]).mag();
        speed[n - 1] = limit_acceleration(speed[n], speed[n - 1], distance,
                                          max_tangential_acceleration);
    }

    Trajectory trajectory{{}};

    Pose initial_pose{points[0], 0};
    Twist initial_twist{derivs1[0].normalized(speed[0]), 0};
    trajectory.AppendInstant(
        RobotInstant{initial_pose, initial_twist, initial_time});

    for (int n = 1; n < num_points; n++) {
        Point deltaPos = points[n] - points[n - 1];

        double distance = deltaPos.mag();

        // Average speed over the interval. We assume constant acceleration,
        // which is true in the limit of small intervals.
        double average_speed = (speed[n] + speed[n - 1]) / 2;
        double interval_time = distance / average_speed;

        if (average_speed == 0 || !std::isfinite(average_speed)) {
            throw std::runtime_error("Invalid average speed");
        }

        if (interval_time <= 0 || !std::isfinite(interval_time)) {
            throw std::runtime_error("Invalid interval time");
        }

        RJ::Time current_time =
            trajectory.last().stamp + RJ::Seconds(interval_time);

        Pose pose{points[n], 0};
        Twist twist{derivs1[n].normalized(speed[n]), 0};

        // Add point n in
        trajectory.AppendInstant(RobotInstant{pose, twist, current_time});
    }
    return std::move(trajectory);
}

}  // namespace Planning
