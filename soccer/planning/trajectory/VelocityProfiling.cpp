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
                           const MotionConstraints& constraints,
                           RJ::Time initial_time) {
    if(path.empty()) return Trajectory{{}};

    Trajectory result({});
    RobotInstant start_instant;

    // Add an initial point to the trajectory so that we keep track of initial
    // velocity
    path.Evaluate(0, &start_instant.pose.position(), &start_instant.velocity.linear());
    start_instant.stamp = initial_time;

    // Scale the velocity so that the initial speed is correct
    start_instant.velocity.linear() = start_instant.velocity.linear().normalized(initial_speed);

    result.AppendInstant(start_instant);

    AppendProfiledVelocity(result, path, final_speed, constraints);
    return result;
}

double limitAccel(double v1, double v2, double deltaX, double maxAccel) {
    double maxV2 = std::sqrt(pow(v1, 2) + 2 * maxAccel * deltaX);
    return std::min(v2, maxV2);
}

void AppendProfiledVelocity(Trajectory& out,
                            const BezierPath& path,
                            double final_speed,
                            const MotionConstraints& constraints) {
    double initial_speed = 0;

    // The rest of this code will assume that we don't need to add the first
    // instant in the trajectory, because it will be at the end of the existing
    // trajectory. Add an instant to ensure this is true when out is empty.
    if (out.empty()) {
        RobotInstant instant;
        instant.stamp = RJ::now();
        path.Evaluate(0, &instant.pose.position());
        out.AppendInstant(instant);
    }

    // Planning starts after `out` ends, so we want to capture the ending speed.
    initial_speed = out.last().velocity.linear().mag();

    // number of points used to interpolate each bezier segment
    constexpr int interpolations = 40;
    // number of cubic bezier segments
    const int num_beziers = path.size();
    // number of points that will be in the final trajectory
    const int num_points = num_beziers * interpolations;

    // Scratch data that we will use later.
    std::vector<Point> points(num_points), derivs1(num_points);
    std::vector<double> curvature(num_points), speed(num_points, constraints.maxSpeed);

    double maxCentripetalAccel = std::min(constraints.maxAcceleration, constraints.maxCentripetalAcceleration);

    //note: these are just suggestions. if they are impossible given MotionConstraints, then we'll limit them
    speed[0] = initial_speed;
    speed[num_points-1] = final_speed;

    // Velocity pass: fill points and calculate maximum velocity given curvature
    // at each point.
    for (int n = 0; n < num_points; n++) {
        double s = n / static_cast<double>(num_points-1);
        path.Evaluate(s, &points[n], &derivs1[n], &curvature[n]);

        assert(curvature[n] >= 0.0);
        assert(!std::isnan(curvature[n]) && !std::isinf(curvature[n]));

        // Centripetal acceleration: a = v^2 / r => v = sqrt(ra)
        if (curvature[n] != 0.0) {
            speed[n] = std::min(speed[n], std::sqrt(maxCentripetalAccel / curvature[n]));
        }
    }

    using std::pow;
    // Acceleration pass: calculate maximum velocity at each point based on
    // acceleration limits forwards in time.
    for (int n = 0; n < num_points-1; n++) {
        double centripetal = speed[n] * speed[n] * curvature[n];
        double maxTanAccelSquared = pow(constraints.maxAcceleration, 2) - pow(centripetal, 2);
        double maxTangentAccel = std::abs(maxTanAccelSquared) < 0.0000001 ? 0.0 : std::sqrt(maxTanAccelSquared);
        assert(!std::isnan(maxTangentAccel) && !std::isinf(maxTangentAccel));
        double distance = (points[n + 1] - points[n]).mag();
        speed[n + 1] = limitAccel(speed[n], speed[n + 1], distance, maxTangentAccel);
    }

    // Decceleration pass: calculate maximum velocity at each point based on
    // acceleration limits backwards in time.
    for (int n = num_points-1; n > 0; n--) {
        double centripetal = speed[n] * speed[n] * curvature[n];
        double maxTanAccelSquared = pow(constraints.maxAcceleration, 2) - pow(centripetal, 2);
        double maxTangentAccel = std::abs(maxTanAccelSquared) < 0.0000001 ? 0.0 : std::sqrt(maxTanAccelSquared);
        assert(!std::isnan(maxTangentAccel) && !std::isinf(maxTangentAccel));
        double distance = (points[n - 1] - points[n]).mag();
        speed[n - 1] = limitAccel(speed[n], speed[n - 1], distance, maxTangentAccel);
    }

    // TODO(Kyle): Allow the user to pass in an initial time. done?
    RJ::Time time = out.last().stamp;

    // We skip the first instant.
    for (int n = 1; n < num_points; n++) {
        double distance = (points[n] - points[n - 1]).mag();
        double vbar = (speed[n] + speed[n - 1]) / 2;
        assert(vbar != 0);
        double t_sec = distance / vbar;
        time = time + RJ::Seconds(t_sec);

        // Add point n in
        out.AppendInstant(RobotInstant{Pose(points[n], 0), Twist(derivs1[n].normalized(speed[n]), 0), time});
    }
}
void PlanAngles(Trajectory& trajectory,
                const RobotInstant& start_instant,
                const AngleFunction& angle_function,
                const RotationConstraints& constraints) {
    if(trajectory.empty()) {
        return;
    }
    trajectory.instant(0).pose.heading() = start_instant.pose.heading();
    trajectory.instant(0).velocity.angular() = start_instant.velocity.angular();

    // Move forwards in time. At each instant, calculate the goal angle and its
    // time derivative, and try to get there with a trapezoidal profile.

    // limit velocity
    // skip the first instant
    for (int i = 0; i < trajectory.num_instants() - 1; i++) {
        const RobotInstant& instant_initial = trajectory.instant(i);
        RobotInstant& instant_final = trajectory.instant(i + 1);
        double deltaTime = RJ::Seconds(instant_final.stamp - instant_initial.stamp).count();
        double delta_angle = fixAngleRadians(angle_function(instant_final)
                                     - instant_initial.pose.heading());
        double vel = delta_angle / deltaTime;
        vel = std::clamp(vel, -constraints.maxSpeed, constraints.maxSpeed);
        instant_final.pose.heading() = instant_initial.pose.heading() + vel * deltaTime;
        instant_final.velocity.angular() = vel;
    }
    // limit acceleration (forward)
    for (int i = 0; i < trajectory.num_instants() - 1; i++) {
        RobotInstant& instant_initial = trajectory.instant(i);
        RobotInstant& instant_final = trajectory.instant(i + 1);
        double w0 = instant_initial.velocity.angular();
        double& wf = instant_final.velocity.angular();
        double deltaTime = RJ::Seconds(instant_final.stamp - instant_initial.stamp).count();
        wf = std::clamp(wf, w0 - constraints.maxAccel * deltaTime, w0 + constraints.maxAccel * deltaTime);
    }
    // limit deceleration (backward)
    for (int i = trajectory.num_instants()-1; i > 0; i--) {
        RobotInstant& instant_initial = trajectory.instant(i);
        RobotInstant& instant_final = trajectory.instant(i - 1);
        double w0 = instant_initial.velocity.angular();
        double& wf = instant_final.velocity.angular();
        double deltaTime = RJ::Seconds(instant_initial.stamp - instant_final.stamp).count();
        wf = std::clamp(wf, w0 - constraints.maxAccel * deltaTime, w0 + constraints.maxAccel * deltaTime);
    }
}
} // namespace Planning
