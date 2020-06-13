#include "AnglePlanning.hpp"

#include "TrapezoidalMotion.hpp"

namespace Planning {

void PlanAngles(Trajectory* trajectory,
                const RobotInstant& start_instant,
                const AngleFunction& angle_function,
                const RotationConstraints& constraints) {
    RJ::Time start_time = start_instant.stamp;

    // Clip the front of the trajectory.
    // TODO(Kyle): Handle this slightly more gracefully, by only profiling the
    //  required portion of the trajectory.
    *trajectory = trajectory->subTrajectory(start_time, trajectory->end_time());

    if (trajectory->num_instants() < 2) {
        trajectory->first() = start_instant;
        trajectory->mark_angles_valid();
        return;
    }
    if (!trajectory->CheckTime(start_time)) {
        throw std::runtime_error("Tried to profile from invalid start time.");
    }

    // Find the target angles for each point along the trajectory. This is
    // required in order to get into the right homotopy class (for example,
    // the tangent-to-path function can either travel in forwards or reverse).
    std::vector<double> target_angles;
    target_angles.resize(trajectory->num_instants());
    Eigen::Vector2d gradient;
    target_angles.at(0) = angle_function(start_instant.linear_motion(),
                                         start_instant.heading(),
                                         &gradient);

    std::vector<double> velocity;
    velocity.resize(trajectory->num_instants());
    velocity.at(0) = start_instant.angular_velocity();

    for (int i = 1; i < trajectory->num_instants(); i++) {
        // Get a temporary copy of the previous state and modify it to have
        // the correct heading (we'll actually edit in-place later).
        LinearMotionInstant instant = trajectory->instant_at(i).linear_motion();
        target_angles.at(i) = angle_function(instant,
                                             target_angles.at(i - 1),
                                             &gradient);
        velocity.at(i) = trajectory->instant_at(i).linear_velocity()
                             .dot(Geometry2d::Point(gradient));
    }

    /*
    // We want to do a best-effort tracking of those values while still ending
    // up in the right place. Do acceleration-limited backwards in time, with
    // a final goal velocity of zero, until we get to a point where we couldn't
    // get back to the initial position in time. From there, fill the rest up
    // with a trapezoidal motion from the beginning.

    Trapezoid::State initial{start_instant.heading(),
                             start_instant.angular_velocity()};

    // The sample to which we need to do a regular trapezoid profile.
    int num_trapezoid_samples = trajectory->num_instants();

    // TODO(Kyle): If we are unable to finish the motion in time at all,
    //  profile all the way until the end, and then append points to the
    //  trajectory containing the remainder of the trapezoid motion.
    for (int i = static_cast<int>(velocity.size() - 1); i > 0; i--) {
        double next_heading = target_angles.at(i);
        double next_velocity = velocity.at(i);

        double target_heading = target_angles.at(i - 1);
        double delta = fixAngleRadians(next_heading - target_heading);

        RJ::Time time_next = trajectory->instant_at(i).stamp;
        RJ::Time time_prev = trajectory->instant_at(i - 1).stamp;
        double dt = RJ::Seconds(time_next - time_prev).count();

        // Fit a quadratic (constant acceleration) with the given final
        // velocity and final position, and attempt to hit the target
        // heading. If acceleration is too high, clamp it and adjust the
        // heading accordingly.
        // d = (vi+vf)t/2 = vf*t - 1/2at^2
        double acceleration =
            2 * (next_velocity * dt - delta) / std::pow(dt, 2);

        // Clamp acceleration to our given limits.
        // TODO(Kyle): Also clamp based on maximum velocity. That shouldn't
        //  be necessary in most situations though, as angle functions are
        //  not expected to have rapid discontinuities (possible exception:
        //  face point, when moving near the point).
        acceleration = std::clamp(acceleration,
                                  -constraints.maxAccel,
                                  constraints.maxAccel);

        double prev_velocity = next_velocity - acceleration * dt;
        double prev_heading =
            next_heading - dt * (next_velocity + prev_velocity) / 2;

        target_angles.at(i - 1) = prev_heading;
        velocity.at(i - 1) = prev_velocity;
    }
     */

    /*
    Trapezoid::State initial{start_instant.heading(),
                             start_instant.angular_velocity()};

    // Trapezoidally profile the beginning of the path.
    for (int i = 0; i < trajectory->num_instants(); i++) {
        RJ::Time time = trajectory->instant_at(i).stamp;
        double time_from_begin =
            RJ::Seconds(time - trajectory->begin_time()).count();

        Trapezoid::State state{target_angles.at(i), velocity.at(i)};
        double trapezoid_time = Trapezoid::timeRemaining(initial,
                                                         state,
                                                         constraints.maxSpeed,
                                                         constraints.maxAccel);
        if (time_from_begin >= trapezoid_time) {
            break;
        }

        state = Trapezoid::predictIn(initial,
                                     state,
                                     constraints.maxSpeed,
                                     constraints.maxAccel,
                                     time_from_begin);
        target_angles.at(i) = state.position;
        velocity.at(i) = state.velocity;
    }
     */

    // Fill in the trajectory
    auto instant_it = trajectory->instants_begin();
    for (int i = 0; i < trajectory->num_instants(); ++i, ++instant_it) {
        instant_it->heading() = target_angles.at(i);
        instant_it->angular_velocity() = velocity.at(i);
    }

    trajectory->mark_angles_valid();
}

} // namespace Planning

