#include "trapezoidal_motion.hpp"

#include <cmath>

namespace planning::Trapezoid {

/**
 * Is the profile "inverted"? That is, do we first accelerate in the positive
 *  or negative direction? true = inverted profile (negative direction)
 */
bool should_flip_acceleration(State initial, State goal, double max_acceleration) {
    // Calculate the distance travelled by a linear ramp from initial velocity
    // to final velocity. If this displacement is more positive than our desired
    // displacement, we should accelerate in the negative direction.
    double delta_velocity = goal.velocity - initial.velocity;
    double displacement = goal.position - initial.position;
    double ramp_time = std::abs(delta_velocity) / max_acceleration;
    return displacement < ramp_time * (initial.velocity + goal.velocity) / 2;
}

void get_motion_times(State initial, State goal, double max_velocity, double max_acceleration,
                      double* end_accel, double* end_constant, double* end) {
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
        double vi_squared = std::pow(max_velocity, 2) + max_acceleration * distance_remaining;

        // We want the positive root, because the negative root will give us a
        // negative time.
        double cutout_time = (max_velocity - std::sqrt(vi_squared)) / max_acceleration;
        time_accel -= cutout_time;
        time_decel -= cutout_time;
    }

    *end_accel = time_accel;
    *end_constant = time_accel + time_constant;
    *end = time_accel + time_constant + time_decel;
}

double time_remaining(State initial, State goal, double max_velocity, double max_acceleration) {
    bool flipped = should_flip_acceleration(initial, goal, max_acceleration);

    if (flipped) {
        initial.position *= -1;
        initial.velocity *= -1;
        goal.position *= -1;
        goal.velocity *= -1;
    }

    double time_accel_end = 0;
    double time_constant_end = 0;
    double time_end = 0;
    get_motion_times(initial, goal, max_velocity, max_acceleration, &time_accel_end,
                     &time_constant_end, &time_end);
    return time_end;
}

State trapezoid_interpolate(State initial, State goal, double max_velocity, double max_acceleration,
                            double time_now, double time_accel_end, double time_constant_end,
                            double time_end) {
    State result{0, 0};

    if (time_now < 0) {
        result = initial;
    } else if (time_now < time_accel_end) {
        result.velocity = initial.velocity + max_acceleration * time_now;
        result.position = initial.position + time_now * (initial.velocity + result.velocity) / 2;
    } else if (time_now < time_constant_end) {
        double distance_accel =
            initial.position + time_accel_end * (initial.velocity + max_velocity) / 2;
        double distance_constant = (time_now - time_accel_end) * max_velocity;
        result.velocity = max_velocity;
        result.position = initial.position + distance_accel + distance_constant;
    } else if (time_now < time_end) {
        double time_remaining = time_end - time_now;
        result.velocity = goal.velocity + max_acceleration * time_remaining;
        result.position = goal.position - time_remaining * (result.velocity + goal.velocity) / 2;
    } else {
        result = goal;
    }

    return result;
}

State predict_in(State initial, State goal, double max_velocity, double max_acceleration,
                 double time_now) {
    bool flipped = should_flip_acceleration(initial, goal, max_acceleration);

    if (flipped) {
        initial.position *= -1;
        initial.velocity *= -1;
        goal.position *= -1;
        goal.velocity *= -1;
    }

    double time_accel_end = 0;
    double time_constant_end = 0;
    double time_end = 0;
    get_motion_times(initial, goal, max_velocity, max_acceleration, &time_accel_end,
                     &time_constant_end, &time_end);

    State result = trapezoid_interpolate(initial, goal, max_velocity, max_acceleration, time_now,
                                         time_accel_end, time_constant_end, time_end);

    if (flipped) {
        result.position *= -1;
        result.velocity *= -1;
    }

    return result;
}

}  // namespace planning::Trapezoid
