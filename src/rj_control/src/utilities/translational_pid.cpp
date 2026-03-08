#include "rj_control/utilities/translational_pid.hpp"

namespace control {

double TranslationalPid::update(
    double setpoint, //NOLINT(bugprone-easily-swappable-parameters)
    double measurement
) {
    double error = setpoint - measurement;
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

} // namespace control