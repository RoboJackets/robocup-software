#include "VelocityProfiling.hpp"

namespace Planning {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

namespace VelocityProfileConstants {
constexpr int interpolationsPerBezier = 40;
constexpr int extraAngleInterpolations = 10;
}  // namespace VelocityProfileConstants
using namespace VelocityProfileConstants;

double limitAccel(double v1, double v2, double deltaX, double maxAccel) {
    if (deltaX < 0) {
        debugThrow("Error in limitAccel() can't handle negative distance");
    } else if (maxAccel < 0) {
        debugThrow("Error in limitAccel() can't handle negative acceleration");
    }
    return std::min(v2, std::sqrt(pow(v1, 2) + 2 * maxAccel * deltaX));
}
double clampAccel(double v1, double v2, double deltaX, double maxAccel) {
    double two_a_dx = std::abs(2 * maxAccel * deltaX);
    double lowerBoundSq = v1 * v1 - two_a_dx;
    double lowerBound = lowerBoundSq > 0 ? std::sqrt(lowerBoundSq) : 0;
    double upperBound = std::sqrt(v1 * v1 + two_a_dx);
    bool thisIsDoAble =
        (v1 > 0 && deltaX > 0) || (v1 < 0 && deltaX < 0) || (v1 * v2 <= 0);
    if (thisIsDoAble) {
        if (deltaX > 0) {
            return std::clamp(v2, lowerBound, upperBound);
        } else {
            return std::clamp(v2, -upperBound, -lowerBound);
        }
    } else {
        // the robot changed direction and did a zig-zag thing with
        // non-constant acceleration (ugh)
        // or deltaX is 0 (shouldn't happen)
        assert(false);
        return v2;
    }
}
Trajectory ProfileVelocity(const BezierPath& path, double initial_speed,
                           double final_speed,
                           const MotionConstraints& constraints,
                           RJ::Time initial_time) {
    if (path.empty()) {
        return Trajectory{{}};
    }
    // number of cubic bezier segments
    const int num_beziers = path.size();
    // number of points that will be in the final trajectory
    // add one to account for the final instant
    const int num_points = num_beziers * interpolationsPerBezier + 1;

    // Scratch data that we will use later.
    std::vector<Point> points(num_points), derivs1(num_points);
    std::vector<double> curvature(num_points),
        speed(num_points, constraints.maxSpeed);

    double maxCentripetalAccel = std::min(
        constraints.maxAcceleration, constraints.maxCentripetalAcceleration);

    // note: these are just suggestions. if they are impossible given
    // MotionConstraints, then we'll limit them
    speed[0] = std::min(speed[0], initial_speed);
    speed[num_points - 1] = std::min(speed[num_points - 1], final_speed);

    // Velocity pass: fill points and calculate maximum velocity given curvature
    // at each point.
    for (int n = 0; n < num_points; n++) {
        double s = n / static_cast<double>(num_points - 1);
        path.Evaluate(s, &points[n], &derivs1[n], &curvature[n]);

        assert(curvature[n] >= 0.0);
        assert(!std::isnan(curvature[n]) && !std::isinf(curvature[n]));

        // Centripetal acceleration: a = v^2 / r => v = sqrt(ra)
        if (curvature[n] != 0.0) {
            speed[n] = std::min(speed[n],
                                std::sqrt(maxCentripetalAccel / curvature[n]));
        }
    }
    //    //if derives1 = 0 at the at the endpoints curvature gets undefined
    //    if(num_points > 1) {
    //        if(derivs1[0].mag() < 1e-6) {
    //            curvature[0] = curvature[1];
    //        }
    //        if(derivs1[num_points-1].mag() < 1e-6) {
    //            curvature[num_points-1] = curvature[num_points-2];
    //        }
    //    }

    using std::pow;
    // Acceleration pass: calculate maximum velocity at each point based on
    // acceleration limits forwards in time.
    for (int n = 0; n < num_points - 1; n++) {
        double centripetal = speed[n] * speed[n] * curvature[n];
        double maxTanAccelSquared =
            pow(constraints.maxAcceleration, 2) - pow(centripetal, 2);
        double maxTangentAccel = std::abs(maxTanAccelSquared) < 1e-6
                                     ? 0.0
                                     : std::sqrt(maxTanAccelSquared);
        assert(!std::isnan(maxTangentAccel) && !std::isinf(maxTangentAccel));
        double distance = (points[n + 1] - points[n]).mag();
        speed[n + 1] =
            limitAccel(speed[n], speed[n + 1], distance, maxTangentAccel);
    }

    // Decceleration pass: calculate maximum velocity at each point based on
    // acceleration limits backwards in time.
    for (int n = num_points - 1; n > 0; n--) {
        double centripetal = speed[n] * speed[n] * curvature[n];
        double maxTanAccelSquared =
            pow(constraints.maxAcceleration, 2) - pow(centripetal, 2);
        double maxTangentAccel = std::abs(maxTanAccelSquared) < 0.0000001
                                     ? 0.0
                                     : std::sqrt(maxTanAccelSquared);
        assert(!std::isnan(maxTangentAccel) && !std::isinf(maxTangentAccel));
        double distance = (points[n - 1] - points[n]).mag();
        speed[n - 1] =
            limitAccel(speed[n], speed[n - 1], distance, maxTangentAccel);
    }

    Trajectory trajectory{{}};
    trajectory.AppendInstant(
        RobotInstant{Pose{points[0], 0},
                     Twist{derivs1[0].normalized(speed[0]), 0}, initial_time});
    for (int n = 1; n < num_points; n++) {
        Point deltaPos = points[n] - points[n - 1];
        double distance;
        double centerAngle = derivs1[n - 1].angleBetween(deltaPos) * 2;
        //        if(curvature[n-1] == 0 || centerAngle < 1e-6) {
        // straight line distance
        distance = deltaPos.mag();
        //        } else {
        //            // calculate arc length
        //            double radius = 1 / curvature[n-1];
        //            //todo(Ethan) fix this. the assertion sometimes failed
        ////            assert(radius > deltaPos.mag() / 2);
        //            radius = std::max(deltaPos.mag()/2, radius);
        //            distance = radius * centerAngle;
        //        }
        double vbar = (speed[n] + speed[n - 1]) / 2;
        assert(vbar != 0);
        double t_sec = distance / vbar;
        assert(t_sec > 1e-12);
        RJ::Time current_time = trajectory.last().stamp + RJ::Seconds(t_sec);
        // Add point n in
        trajectory.AppendInstant(RobotInstant{
            Pose(points[n], 0), Twist(derivs1[n].normalized(speed[n]), 0),
            current_time});
    }
    return std::move(trajectory);
}

/*
 * Return to the current position with angular velocity = 0
 *
 * current angle     (stoppingAngle)          0 vel
 * |-------------------------------------------|
 * >>>>>>>>>>>>>> stopping >>>>>>>>>>>>>>>>>>>>>
 *                      <<<<<< accelerating <<<<
 * <<<<<< stopping <<<<<
 */
void appendStop(std::vector<double>& angles, std::vector<double>& angleVels,
                double maxSpeed, double maxAccel) {
    assert(maxSpeed > 1e-12);
    assert(maxAccel > 1e-12);
    assert(std::abs(angleVels.back()) > 1e-12);
    if (std::abs(angleVels.back()) > 1e-12) {
        double accel = maxAccel * (angleVels.back() > 0 ? -1 : 1);
        double stoppingAngle = -std::pow(angleVels.back(), 2) / (2 * accel);
        double goal = angles.back();
        // stopping
        angles.push_back(goal + stoppingAngle);
        angleVels.push_back(0);
        // accelerating
        angles.push_back(goal + stoppingAngle / 2);
        double speedUncapped =
            std::sqrt(2 * maxAccel * std::abs(stoppingAngle / 2));
        double returnSpeedMax = std::min(maxSpeed, speedUncapped);
        angleVels.push_back(returnSpeedMax * (stoppingAngle > 0 ? -1 : 1));
        // stopping
        angles.push_back(goal);
        angleVels.push_back(0);
    }
}
/**
 * Stop as soon as possible. we will end up at a different position:
 * current angle     (stoppingAngle)          0 vel
 * |-------------------------------------------|
 * >>>>>>>>>>>>>> stopping >>>>>>>>>>>>>>>>>>>>>
 */
void appendStopWithDrift(std::vector<double>& angles,
                         std::vector<double>& angleVels, double maxSpeed,
                         double maxAccel) {
    assert(maxSpeed > 1e-12);
    assert(maxAccel > 1e-12);
    assert(std::abs(angleVels.back()) > 1e-12);
    if (std::abs(angleVels.back()) > 1e-12) {
        double accel = maxAccel * (angleVels.back() > 0 ? -1 : 1);
        double stoppingAngle = -std::pow(angleVels.back(), 2) / (2 * accel);
        angles.push_back(angles.back() + stoppingAngle);
        angleVels.push_back(0);
    }
}
// todo: do velocity profile angles
void PlanAngles(Trajectory& trajectory, const RobotInstant& start_instant,
                const AngleFunction& angle_function,
                const RotationConstraints& constraints) {
    if (trajectory.empty()) {
        return;
    }
    auto instants_it = trajectory.instants_begin();
    instants_it->pose.heading() = start_instant.pose.heading();
    instants_it->velocity.angular() =
        std::clamp(start_instant.velocity.angular(), -constraints.maxSpeed,
                   constraints.maxSpeed);
    RobotInstant instant_before = *instants_it;
    ++instants_it;
    for (int i = 1; instants_it != trajectory.instants_end(); i++) {
        RobotInstant& instant_after = *instants_it;
        double deltaTime =
            RJ::Seconds(instant_after.stamp - instant_before.stamp).count();
        instant_after.pose.heading() = angle_function(instant_before);
        double deltaAngle = fixAngleRadians(instant_after.pose.heading() -
                                            instant_before.pose.heading());
        instant_after.velocity.angular() = deltaAngle / deltaTime;
        instant_before = instant_after;
        ++instants_it;
    }
}
}  // namespace Planning
