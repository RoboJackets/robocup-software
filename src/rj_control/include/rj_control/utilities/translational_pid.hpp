/**
 * @file translational_pid.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The translational PID calculates classical error (i.e. setpoint - measured)
 * @version 0.1
 * @date 2026-01-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "pid.hpp"

namespace control {

class TranslationalPid : public Pid {
public:
    /**
     * @brief Construct a new Translational Pid object
     * 
     * @param maximum_output The maximum output of the translational pid (in m/s)
     * @param kp The proportional gain
     * @param ki The integral gain
     * @param kd The derivative gain
     */
    TranslationalPid(
        double maximum_output, // NOLINT(bugprone-easily-swappable-parameters)
        double kp, // NOLINT(readability-identifier-length)
        double ki, // NOLINT(readability-identifier-length)
        double kd // NOLINT(readability-identifier-length)
    ) : Pid(maximum_output, kp, ki, kd) {}

    /**
     * @brief Perform a translational pid update
     * 
     * @param setpoint
     * @param measurement 
     * @return double 
     */
    double update(double setpoint, double measurement) override;
};

} // namespace control