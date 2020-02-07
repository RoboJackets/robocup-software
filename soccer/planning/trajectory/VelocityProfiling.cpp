#include "VelocityProfiling.hpp"

namespace Planning {

using Geometry2d::Point;
using Geometry2d::Pose;
using Geometry2d::Twist;

namespace VelocityProfileConstants{
    //todo(Ethan) increase
    constexpr int inerpolationsPerBezier = 10;
    constexpr int extraAngleInterpolations = 10;
}
using namespace VelocityProfileConstants;

double limitAccel(double v1, double v2, double deltaX, double maxAccel) {
    if(deltaX < 0) {
        debugThrow("Error in limitAccel() can't handle negative distance");
    } else if (maxAccel < 0) {
        debugThrow("Error in limitAccel() can't handle negative acceleration");
    }
    return std::min(v2, std::sqrt(pow(v1, 2) + 2 * maxAccel * deltaX));
}
double clampAccel(double v1, double v2, double deltaX, double maxAccel) {
    double two_a_dx = std::abs(2 * maxAccel * deltaX);
    double lowerBoundSq = v1*v1 - two_a_dx;
    double lowerBound = lowerBoundSq > 0 ? std::sqrt(lowerBoundSq) : 0;
    double upperBound = std::sqrt(v1*v1 + two_a_dx);
    bool thisIsDoAble = (v1 > 0 && deltaX > 0) || (v1 < 0 && deltaX < 0) || (v1*v2 <= 0);
    if(thisIsDoAble) {
        if(deltaX > 0) {
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
Trajectory ProfileVelocity(const BezierPath& path,
                           double initial_speed,
                           double final_speed,
                            const MotionConstraints& constraints,
                            RJ::Time initial_time) {
    if(path.empty()) {
        return Trajectory{{}};
    }
    // number of cubic bezier segments
    const int num_beziers = path.size();
    // number of points that will be in the final trajectory
    // add one to account for the final instant
    const int num_points = num_beziers * inerpolationsPerBezier + 1;

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
    for (int n = 0; n < num_points-1; n++) {
        double centripetal = speed[n] * speed[n] * curvature[n];
        double maxTanAccelSquared = pow(constraints.maxAcceleration, 2) - pow(centripetal, 2);
        double maxTangentAccel = std::abs(maxTanAccelSquared) < 1e-6 ? 0.0 : std::sqrt(maxTanAccelSquared);
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
    for (int n = 1; n < num_points; n++) {
        Point deltaPos = points[n]-points[n-1];
        double distance;
        double centerAngle = derivs1[n-1].angleBetween(deltaPos) * 2;
//        if(curvature[n-1] == 0 || centerAngle < 1e-6) {
            // straight line distance
            distance = deltaPos.mag();
//        } else {
//            // calculate arc length
//            double radius = 1 / curvature[n-1];
//            //todo(Ethan) fix this
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
        trajectory.AppendInstant(RobotInstant{Pose(points[n], 0), Twist(derivs1[n].normalized(speed[n]), 0), current_time});
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
void appendStop(std::vector<double>& angles, std::vector<double>& angleVels, double maxSpeed, double maxAccel) {
    assert(maxSpeed > 1e-12);
    assert(maxAccel > 1e-12);
    assert(std::abs(angleVels.back()) > 1e-12);
    if(std::abs(angleVels.back()) > 1e-12) {
        double accel = maxAccel * (angleVels.back() > 0 ? -1 : 1);
        double stoppingAngle = -std::pow(angleVels.back(), 2) / (2 * accel);
        double goal = angles.back();
        // stopping
        angles.push_back(goal + stoppingAngle);
        angleVels.push_back(0);
        // accelerating
        angles.push_back(goal + stoppingAngle / 2);
        double speedUncapped = std::sqrt(2 * maxAccel * std::abs(stoppingAngle/2));
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
void appendStopWithDrift(std::vector<double>& angles, std::vector<double>& angleVels, double maxSpeed, double maxAccel) {
    assert(maxSpeed > 1e-12);
    assert(maxAccel > 1e-12);
    assert(std::abs(angleVels.back()) > 1e-12);
    if(std::abs(angleVels.back()) > 1e-12) {
        double accel = maxAccel * (angleVels.back() > 0 ? -1 : 1);
        double stoppingAngle = -std::pow(angleVels.back(), 2) / (2 * accel);
        angles.push_back(angles.back() + stoppingAngle);
        angleVels.push_back(0);
    }
}
void PlanAngles(Trajectory& trajectory,
                const RobotInstant& start_instant,
                const AngleFunction& angle_function,
                const RotationConstraints& constraints) {
    if(trajectory.empty()) {
        return;
    }
    std::vector<double> angles(trajectory.num_instants(), 0.0);
    std::vector<double> angleVels(trajectory.num_instants(), 0.0);
    const double maxSpeed = constraints.maxSpeed;
    const double maxAccel = constraints.maxAccel;
    angles[0] = start_instant.pose.heading();
    angleVels[0] = std::clamp(start_instant.velocity.angular(), -maxSpeed, maxSpeed);

    // Move forwards in time. At each instant, calculate the goal angle and its
    // time derivative, and try to get there with a trapezoidal profile.

    // limit velocity
    // skip the first instant
    auto instants_it = trajectory.instants_begin();
    RobotInstant instant_before = *instants_it;
    ++instants_it;
    for (int i = 1; instants_it != trajectory.instants_end(); i++) {
        assert(i < angles.size());
        RobotInstant instant_after = *instants_it;
        double deltaTime = RJ::Seconds(instant_after.stamp - instant_before.stamp).count();
        double delta_angle = fixAngleRadians(angle_function(instant_after) - angles[i - 1]);
        assert(deltaTime > 0);
        double velAvg = delta_angle / deltaTime;
        // assuming constant acceleration: vf = vAvg + vAvg - v0
        // but, using this vf will cause oscillation.
        // |-----|-----|-----|------.......
        //   ^ if we accelerate too much here,
        //          ^ then we have to slow down here to meet time constraints
        //                ^ then we will speed up again here
        //                       ^ ... continues oscillating
        // so we use the value in the middle of vf and vAvg
        angleVels[i] = std::clamp(velAvg + (velAvg - angleVels[i-1])/2, -maxSpeed, maxSpeed);
        velAvg = (angleVels[i-1] + angleVels[i]) / 2;
        angles[i] = angles[i-1] + velAvg * deltaTime;
        instant_before = instant_after;
        ++instants_it;
    }
    // Right now, when the angular velocity changes direction there is a cusp
    // for each cusp, we have to do a velocity profile on either side to fix it
    // profile on both sides, so we don't forget the point where direction changes
    instants_it = trajectory.instants_begin();
    instant_before = *instants_it;
    ++instants_it;
    for(int i = 1; i < angleVels.size();) {
        double direction = std::abs(angleVels[i-1]) > 1e-12 ? angleVels[i-1] : angleVels[i];
        if(std::abs(direction) < 1e-12) {
            i++;
            instant_before = *instants_it;
            ++instants_it;
            continue;
        }
        // limit acceleration (forward constraints)
        int j = i;
        for (; j < angles.size() && angleVels[j] * direction >= 0; j++) {
            assert(instants_it != trajectory.instants_end());
            RobotInstant instant_after = *instants_it;
            double deltaTime = RJ::Seconds(instant_after.stamp-instant_before.stamp).count();
            double deltaAngle = fixAngleRadians(angles[j] - angles[j-1]);
            angleVels[j] = clampAccel(angleVels[j-1], angleVels[j], deltaAngle, maxAccel);
            double maxDeltaVel = maxAccel*deltaTime;
            angleVels[i] = std::clamp(angleVels[i], angleVels[i-1] - maxDeltaVel, angleVels[i-1] + maxDeltaVel);
            instant_before = instant_after;
            ++instants_it;
        }
        // at this point j is invalid (either off the end of the vector or in
        // a position with opposite direction)
        auto instants_it_at_j = instants_it;
        // set the velocity at the cusp to 0 (also assumes the target angular
        // velocity at the end of the path is 0)
        angleVels[j-1] = 0;
        // limit deceleration (backward constraints)
        --instants_it;
        instant_before = *instants_it;
        if(instants_it != trajectory.instants_begin()) --instants_it;
        for (int k = j-1; k >= i; k--) {
            RobotInstant instant_after = *instants_it;
            double deltaTime = RJ::Seconds(instant_before.stamp-instant_after.stamp).count();
            double deltaAngle = fixAngleRadians(angles[k-1] - angles[k]);
            angleVels[k-1] = -clampAccel(-angleVels[k], -angleVels[k-1], deltaAngle, maxAccel);
            double maxDeltaVel = maxAccel*deltaTime;
            angleVels[k-1] = std::clamp(angleVels[k-1], angleVels[k] - maxDeltaVel, angleVels[k] + maxDeltaVel);
            instant_before = instant_after;
            if(instants_it != trajectory.instants_begin()) {
                --instants_it;
            }
        }
        i = j;
        instants_it = instants_it_at_j;
        assert(instants_it != trajectory.instants_begin());
        instant_before = *std::prev(instants_it);
    }

    //update the instants in the trajectory with heading and angular velocity
    trajectory.first().pose.heading() = angles[0];
    trajectory.first().velocity.angular() = angleVels[0];
    instants_it = trajectory.instants_begin();
    ++instants_it;
    int angleIdx = 1;
    for (; instants_it != trajectory.instants_end(); ++instants_it, ++angleIdx) {
        double deltaTime = RJ::Seconds(instants_it->stamp - std::prev(instants_it)->stamp).count();
        double maxDeltaVel = maxAccel * deltaTime;
        //todo(Ethan) is the clamp necessary? explain.
        angleVels[angleIdx] = instants_it->velocity.angular() = std::clamp(angleVels[angleIdx], angleVels[angleIdx-1] - maxDeltaVel, angleVels[angleIdx-1] + maxDeltaVel);
        double avgVel = (angleVels[angleIdx-1] + angleVels[angleIdx])/2;
        angles[angleIdx] = instants_it->pose.heading() = angles[angleIdx-1] + avgVel * deltaTime;
    }
    assert(angleIdx == trajectory.num_instants());

    // If the input trajectory doesn't provide enough time to get to the
    // target angle and velocity we need to stick some extra instants on the end

    // add more instants to get to the target heading at the end
    double angle_initial = angles.back();
    double angle_final = angle_function(trajectory.last());
    double angleLeft = fixAngleRadians(angle_final - angle_initial);
    constexpr double minAngleDelta = 1e-12;
    if(std::abs(angleLeft) > minAngleDelta) {
        //if we are going the wrong direction, we need to stop first
        if(angleLeft * angleVels.back() < -1e-12) {
            appendStopWithDrift(angles, angleVels, constraints.maxSpeed, maxAccel);
            //update these because we have drifted a bit
            angle_initial = angles.back();
            angleLeft = fixAngleRadians(angle_final - angle_initial);
        }
        //add extra angles at max speed
        int sizeBeforePivot = angles.size();
        for(int i = 1; i <= extraAngleInterpolations; i++) {
            double percentProgress = (double)i / extraAngleInterpolations;
            double currentAngle = percentProgress * angleLeft + angle_initial;
            if(i == extraAngleInterpolations) {
                currentAngle = angle_final;
            }
            double angleDelta =  fixAngleRadians(currentAngle - angles.back());
            if(std::abs(angleDelta) > minAngleDelta) {
                angles.push_back(currentAngle);
                angleVels.push_back(maxSpeed * (angleDelta < 0 ? -1 : 1));
            }
        }
        //assume target velocity = 0
        angleVels.back() = 0;
        //extra angles--backward pass (limited to the extra instants region)
        for(int i = angles.size()-1; i > sizeBeforePivot; i--) {
            double angleDelta =  fixAngleRadians(angles[i-1] - angles[i]);
            angleVels[i-1] = -clampAccel(-angleVels[i],
                    -angleVels[i-1], angleDelta, maxAccel);
        }
        //extra angles--forward pass
        for(int i = sizeBeforePivot; i < angles.size(); i++) {
            double angleDelta =  fixAngleRadians(angles[i] - angles[i-1]);
            angleVels[i] = clampAccel(angleVels[i-1], angleVels[i], angleDelta, maxAccel);
        }
    }
    //if we are still moving, let's stop
    if(std::abs(angleVels.back()) > 1e-12) {
        appendStop(angles, angleVels, maxSpeed, maxAccel);
    }
    //add extra instants to the trajectory to get to the target angle
    assert(angleIdx != 0);
    assert(angles.size() == angleVels.size());
    for (; angleIdx < angles.size(); ++angleIdx) {
        RobotInstant newInstant{trajectory.last()};
        newInstant.pose.heading() = angles[angleIdx];
        newInstant.velocity.angular() = angleVels[angleIdx];
        double deltaAngle = fixAngleRadians(angles[angleIdx] - trajectory.last().pose.heading());
        double angleVelAvg = (angleVels[angleIdx] + angleVels[angleIdx-1]) / 2;
        assert(deltaAngle * angleVels[angleIdx] > -1e-12);
        if(std::abs(angleVelAvg) > 1e-9 && std::abs(deltaAngle) > 1e-9) {
            double deltaTime = deltaAngle / angleVelAvg;
            newInstant.stamp = trajectory.last().stamp + RJ::Seconds(deltaTime);
            trajectory.AppendInstant(newInstant);
        }
    }

}
} // namespace Planning
