#include "TrapezoidalMotion.hpp"

#include <cmath>

namespace Planning::Trapezoid {

/**
 * Is the profile "inverted"? That is, do we first accelerate in the positive
 *  or negative direction? true = inverted profile (negative direction)
 */
bool shouldFlipAcceleration(State initial,
                            State goal,
                            double max_acceleration) {
    // Calculate the distance travelled by a linear ramp from initial velocity
    // to final velocity. If this displacement is more positive than our desired
    // displacement, we should accelerate in the negative direction.
    double delta_velocity = goal.velocity - initial.velocity;
    double displacement = goal.position - initial.position;
    double ramp_time = std::abs(delta_velocity) / max_acceleration;
    return displacement < ramp_time * (initial.velocity + goal.velocity) / 2;
}

void getMotionTimes(State initial,
                    State goal,
                    double max_velocity,
                    double max_acceleration,
                    double* end_accel,
                    double* end_constant,
                    double* end) {
    // Calculate the time to ramp up to max speed and the time to get from max
    // speed down to the goal speed.
    double time_accel = std::abs(max_velocity - initial.velocity) / max_acceleration;
    double time_decel = std::abs(max_velocity - goal.velocity) / max_acceleration;

    double distance_accel = time_accel * (initial.velocity + max_velocity) / 2;
    double distance_decel = time_decel * (goal.velocity + max_velocity) / 2;

    // Calculate the distance remaining to be taken up by the constant-velocity
    // section
    double displacement = goal.position - initial.position;
    double distance_remaining = displacement - distance_accel - distance_decel;

    double time_constant = distance_remaining / max_velocity;
    if (distance_remaining < 0) {
        time_constant = 0;

        // We need to cut out distance_remaining / 2 from both the acceleration
        // and the deceleration movements. Solve for final velocity using
        // kinematics equations:
        // vf^2 - vi^2 = 2ad, where d = distance_remaining/2
        double vi_squared = std::pow(max_velocity, 2)
                            + max_acceleration * distance_remaining;

        // We want the positive root, because the negative root will give us a
        // negative time.
        double cutout_time =
            (max_velocity - std::sqrt(vi_squared)) / max_acceleration;
        time_accel -= cutout_time;
        time_decel -= cutout_time;
    }

    *end_accel = time_accel;
    *end_constant = time_accel + time_constant;
    *end = time_accel + time_constant + time_decel;
}

double timeRemaining(State initial,
                     State goal,
                     double max_velocity,
                     double max_acceleration) {
    bool flipped = shouldFlipAcceleration(initial,
                                          goal,
                                          max_acceleration);

    if (flipped) {
        initial.position *= -1;
        initial.velocity *= -1;
        goal.position *= -1;
        goal.velocity *= -1;
    }

    double time_accel_end = 0;
    double time_constant_end = 0;
    double time_end = 0;
    getMotionTimes(initial,
                   goal,
                   max_velocity,
                   max_acceleration,
                   &time_accel_end,
                   &time_constant_end,
                   &time_end);
    return time_end;
}

State trapezoidInterpolate(State initial,
                           State goal,
                           double max_velocity,
                           double max_acceleration,
                           double time_now,
                           double time_accel_end,
                           double time_constant_end,
                           double time_end) {
    State result{0, 0};

    if (time_now < 0) {
        result = initial;
    } else if (time_now < time_accel_end) {
        result.velocity = initial.velocity + max_acceleration * time_now;
        result.position =
            initial.position +
            time_now * (initial.velocity + result.velocity) / 2;
    } else if (time_now < time_constant_end) {
        double distance_accel =
            initial.position +
            time_accel_end * (initial.velocity + max_velocity) / 2;
        double distance_constant = (time_now - time_accel_end) * max_velocity;
        result.velocity = max_velocity;
        result.position = initial.position + distance_accel + distance_constant;
    } else if (time_now < time_end) {
        double time_remaining = time_end - time_now;
        result.velocity = goal.velocity + max_acceleration * time_remaining;
        result.position =
            goal.position -
            time_remaining * (result.velocity + goal.velocity) / 2;
    } else {
        result = goal;
    }

    return result;
}

State predictIn(State initial,
                State goal,
                double max_velocity,
                double max_acceleration,
                double time_now) {
    bool flipped = shouldFlipAcceleration(initial,
                                          goal,
                                          max_acceleration);

    if (flipped) {
        initial.position *= -1;
        initial.velocity *= -1;
        goal.position *= -1;
        goal.velocity *= -1;
    }

    double time_accel_end = 0;
    double time_constant_end = 0;
    double time_end = 0;
    getMotionTimes(initial,
                   goal,
                   max_velocity,
                   max_acceleration,
                   &time_accel_end,
                   &time_constant_end,
                   &time_end);

    State result = trapezoidInterpolate(initial,
                                        goal,
                                        max_velocity,
                                        max_acceleration,
                                        time_now,
                                        time_accel_end,
                                        time_constant_end,
                                        time_end);

    if (flipped) {
        result.position *= -1;
        result.velocity *= -1;
    }

    return result;
}

std::optional<State> predictWithExactEndTime(State initial,
                                             State goal,
                                             double max_velocity,
                                             double max_acceleration,
                                             double time_now,
                                             double request_end) {
    bool flipped = shouldFlipAcceleration(initial, goal, max_acceleration);

    if (flipped) {
        initial.position *= -1;
        initial.velocity *= -1;
        goal.position *= -1;
        goal.velocity *= -1;
    }

    // First, get the fastest possible timings.
    double fast_t1 = 0;
    double fast_t2 = 0;
    double fast_end = 0;

    getMotionTimes(initial, goal, max_velocity, max_acceleration, &fast_t1, &fast_t2, &fast_end);
    double fast_constant_time = fast_t2 - fast_t1;
    double fast_vmax = initial.velocity + max_acceleration * fast_t1;

    if (fast_end > request_end) {
        // We have requested to complete the motion faster than physically possible, given the constraints.
        // Return nullopt.
        return std::nullopt;
    }

    // We will make up for slack time by lowering the maximum velocity. To do
    // this, we will find the velocity upper bound that would create a profile
    // that wastes exactly the correct amount of time. The trapezoid/triangle
    // that we "slice off" of the top should take time equal to this slack time.
    double slack_time = request_end - fast_end;

    // For these calculations, let...
    // - a_m is the maximum acceleration
    // - t_c is the _original_ duration at constant velocity
    // - v_m is the _original_ max velocity
    //
    // Decision variable is dv (delta-v), decrease in maximum velocity.
    //
    // If we keep the same profile but "cut off" the top by dv, we reduce total
    // distance by dv*t_c for the constant region and 2*1/2a_m t_a^2 = dv^2/a_m
    // for the accelerating regions. so, dd (delta distance) is:
    // dd = dv*t_c + dv^2/a_m
    //
    // We'll reinsert that distance into the constant region at velocity v_m-dv.
    // This increases the total time by dt (slack time):
    // dt = dd/(v_m-dv) => dt * (v_m-dv) = dv*t_c + dv^2/a_m
    // 0 = (1/a_m) dv^2 + (dt+t_c) dv - (dt*v_m)
    // We can solve this quadratic - we will always want the smallest positive
    // solution, which is the larger solution.
    double qa = 1 / max_acceleration;
    double qb = slack_time + fast_constant_time;
    double qc = -slack_time * fast_vmax;

    // Solve quadratic.
    double dv = (-qb + std::sqrt(qb * qb - 4 * qa * qc)) / (2 * qa);

    double vmax = fast_vmax - dv;
    double t1 = fast_t1 - dv / max_acceleration;

    // Calculate info for truncated profile.
    double truncated_accel_time = (vmax - initial.velocity) / max_acceleration;
    double truncated_decel_time = (vmax - goal.velocity) / max_acceleration;
    double truncated_constant_time =
        fast_end - truncated_accel_time - truncated_decel_time;
    double truncated_constant_distance = vmax * truncated_constant_time;
    double cutoff_profile_distance =
        0.5 * (initial.velocity + vmax) * truncated_accel_time +
        vmax * truncated_constant_time +
        0.5 * (goal.velocity + vmax) * truncated_decel_time;

    double additional_constant_distance =
        goal.position - initial.position - cutoff_profile_distance;
    double constant_time =
        (truncated_constant_distance + additional_constant_distance) / vmax;
    double t2 = t1 + constant_time;
    double time_end =
        truncated_accel_time + constant_time + truncated_decel_time;

    State result = trapezoidInterpolate(initial,
                                        goal,
                                        vmax,
                                        max_acceleration,
                                        time_now,
                                        t1,
                                        t2,
                                        time_end);

    if (flipped) {
        result.position *= -1;
        result.velocity *= -1;
    }

    return result;
}

} // namespace Planning::Trapezoid

