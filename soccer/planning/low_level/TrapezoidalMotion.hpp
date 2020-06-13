#pragma once

#include <optional>

namespace Planning::Trapezoid {

struct State {
    double position;
    double velocity;
};

double timeRemaining(State initial,
                     State goal,
                     double max_velocity,
                     double max_acceleration);

State predictIn(State initial,
                State goal,
                double max_velocity,
                double max_acceleration,
                double time_now);

std::optional<State> predictWithExactEndTime(State initial,
                                             State goal,
                                             double max_velocity,
                                             double max_acceleration,
                                             double time_now,
                                             double time_end);

} // namespace Planning::Trapezoid