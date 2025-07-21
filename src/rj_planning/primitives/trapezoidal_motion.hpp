#pragma once

#include <optional>

namespace planning::Trapezoid {

struct State {
    double position;
    double velocity;
};

/**
 * @brief Calculate the time remaining using a bang-bang profile to get to the
 * specified state from any initial state (other than greater-than-max
 * velocity).
 *
 * @param initial The current state of the system.
 * @param goal The final state of the system.
 * @param max_velocity The maximum allowed velocity. initial.velocity must be
 * within this speed.
 * @return The minimum time required to go from initial to goal.
 */
double time_remaining(State initial, State goal, double max_velocity,
                     double max_acceleration);

/**
 * @brief Use trapezoidal profiling to get an intermediate state on the way to a
 * goal, after a set amount of time.
 *
 * @param initial The current state of the system.
 * @param goal The final goal state of the system.
 * @param max_velocity The maximum allowed velocity. initial.velocity must be
 * within this speed.
 * @param time_now The time at which to query the profile.
 * @return The minimum time required to go from initial to goal.
 */
State predict_in(State initial, State goal, double max_velocity,
                double max_acceleration, double time_now);

}  // namespace planning::Trapezoid