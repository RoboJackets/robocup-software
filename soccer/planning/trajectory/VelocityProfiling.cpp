#include "VelocityProfiling.hpp"
#include <motion/TrapezoidalMotion.hpp>
#include <Utils.hpp>

namespace Planning {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

inline double limitAccel(double v1, double v2, double deltaX, double maxAccel) {
    return std::min(v2, std::sqrt(pow(v1, 2) + 2 * maxAccel * deltaX));
}

Trajectory ProfileVelocity(const BezierPath& path,
                           double initial_speed,
                           double final_speed,
                            const MotionConstraints& constraints,
                            RJ::Time initial_time) {
    if(path.empty()) {
        return Trajectory{{}};
    }
    // number of points used to interpolate each bezier segment
    constexpr int interpolations = 40;
    // number of cubic bezier segments
    const int num_beziers = path.size();
    // number of points that will be in the final trajectory
    // add one to account for the final instant
    const int num_points = num_beziers * interpolations + 1;

    // Scratch data that we will use later.
    std::vector<Point> points(num_points), derivs1(num_points);
    std::vector<double> curvature(num_points), speed(num_points, constraints.maxSpeed);

    double maxCentripetalAccel = std::min(constraints.maxAcceleration, constraints.maxCentripetalAcceleration);

    //note: these are just suggestions. if they are impossible given MotionConstraints, then we'll limit them
    speed[0] = std::min(speed[0], initial_speed);
    speed[num_points-1] = std::min(speed[num_points-1], final_speed);

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

    Trajectory trajectory{{}};
    trajectory.AppendInstant(RobotInstant{Pose{points[0], 0}, Twist{derivs1[0].normalized(speed[0]), 0}, initial_time});
    RJ::Time current_time = initial_time;
    for (int n = 1; n < num_points; n++) {
        double distance = (points[n] - points[n - 1]).mag();
        double vbar = (speed[n] + speed[n - 1]) / 2;
        assert(vbar != 0);
        double t_sec = distance / vbar;
        current_time = current_time + RJ::Seconds(t_sec);
        // Add point n in
        trajectory.AppendInstant(RobotInstant{Pose(points[n], 0), Twist(derivs1[n].normalized(speed[n]), 0), current_time});
    }
    return std::move(trajectory);
}
void PlanAngles(Trajectory& trajectory,
                const RobotInstant& start_instant,
                const AngleFunction& angle_function,
                const RotationConstraints& constraints) {
    if(trajectory.empty()) {
        return;
    }
    trajectory.first().pose.heading() = start_instant.pose.heading();
    trajectory.first().velocity.angular() = start_instant.velocity.angular();

    // Move forwards in time. At each instant, calculate the goal angle and its
    // time derivative, and try to get there with a trapezoidal profile.

    // limit velocity
    // skip the first instant
    auto instants_it = trajectory.instants_begin();
    RobotInstant instant_initial = *instants_it;
    ++instants_it;
    while (instants_it != trajectory.instants_end()) {
        RobotInstant& instant_final = *instants_it;
        double deltaTime = RJ::Seconds(instant_final.stamp - instant_initial.stamp).count();
        double delta_angle = fixAngleRadians(angle_function(instant_final)
                                     - instant_initial.pose.heading());
        double vel = delta_angle / deltaTime;
//        vel = std::clamp(vel, -constraints.maxSpeed, constraints.maxSpeed);
        instant_final.pose.heading() = angle_function(instant_final);//instant_initial.pose.heading() + vel * deltaTime;
        instant_final.velocity.angular() = vel;
        instant_initial = instant_final;
        ++instants_it;
    }
    instants_it = trajectory.instants_begin();
    instant_initial = *instants_it;
    ++instants_it;
    // limit acceleration (forward)
    while (instants_it != trajectory.instants_end()) {
        RobotInstant& instant_final = *instants_it;
        double w0 = instant_initial.velocity.angular();
        double& wf = instant_final.velocity.angular();
        double deltaTime = RJ::Seconds(instant_final.stamp - instant_initial.stamp).count();
        wf = std::clamp(wf, w0 - constraints.maxAccel * deltaTime, w0 + constraints.maxAccel * deltaTime);
        instant_initial = instant_final;
        ++instants_it;
    }
    instants_it = trajectory.instants_end();
    --instants_it;
    instant_initial = *instants_it;
    // limit deceleration (backward)
    while(instants_it != trajectory.instants_begin()) {
        --instants_it;
        RobotInstant& instant_final = *instants_it;
        double w0 = instant_initial.velocity.angular();
        double& wf = instant_final.velocity.angular();
        double deltaTime = RJ::Seconds(instant_initial.stamp - instant_final.stamp).count();
        wf = std::clamp(wf, w0 - constraints.maxAccel * deltaTime, w0 + constraints.maxAccel * deltaTime);
        instant_initial = instant_final;
    }

    //add more instants to get to the target heading
    double angleLeft = fixAngleRadians(angle_function(trajectory.last()) - trajectory.last().pose.heading());
    if(angleLeft > 1e-5) {
        constexpr int extra_interpolations = 20;
        std::list<RobotInstant> extra_instants....
    }
}
} // namespace Planning
