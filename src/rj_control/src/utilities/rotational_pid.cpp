#include "rj_control/utilities/rotational_pid.hpp"

namespace control {

double RotationalPid::update(
    double setpoint,
    double measurement
) {
    double error = wrap_to_pi(setpoint - measurement);
    last_error_ = error;

    // Calculate the proportional term
    double p_term = error * kp_;
    if (p_limit_.has_value()) {
        if (std::abs(p_term) > p_limit_.value()) {
            p_term > 0.0 ? p_term = p_limit_.value() : p_term = -p_limit_.value();
        }
    }

    // Calculate the integral term
    integral_term_ = integral_term_ + error * ki_;
    if (i_limit_.has_value()) {
        if (std::abs(integral_term_) > i_limit_.value()) {
            integral_term_ > 0.0 ? integral_term_ = i_limit_.value() : integral_term_ = -i_limit_.value();
        }
    }

    // Calculate the derivative term
    double d_term = previous_measurement_.has_value() ? (measurement - previous_measurement_.value()) * kd_ : 0.0;
    previous_measurement_ = measurement;
    if (d_limit_.has_value()) {
        if (std::abs(d_term) > d_limit_.value()) {
            d_term > 0.0 ? d_term = d_limit_.value() : d_term = -d_limit_.value();
        }
    }

    return std::clamp(p_term + integral_term_ + d_term, -maximum_output_, maximum_output_);
}

double RotationalPid::wrap_to_pi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace control