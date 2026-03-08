/**
 * @file rotational_pid.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The rotational PID calculates angular error (wrapped between -pi and pi)
 * @version 0.1
 * @date 2026-01-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "pid.hpp"

namespace control {

class RotationalPid : public Pid {
public:
    /**
     * @brief Construct a new Rotational Pid object
     * 
     * @param maximum_output The maximum output from the pid controller (in rad/s)
     * @param kp The proportional gain
     * @param ki The integral gain
     * @param kd The derivative gain
     */
    RotationalPid(
        double maximum_output, // NOLINT(bugprone-easily-swappable-parameters)
        double kp, // NOLINT(readability-identifier-length)
        double ki, // NOLINT(readability-identifier-length)
        double kd // NOLINT(readability-identifier-length)
    ) : Pid(maximum_output, kp, ki, kd) {}

    /**
     * @brief Performa PID update using rotational error
     * 
     * @param setpoint The target heading
     * @param measurement The measured heading
     * @return double The setpoint velocity (in rad/s)
     */
    double update(double setpoint, double measurement) override;

private:
    /**
     * @brief Wrap a given angle between -pi and pi
     * 
     * @param angle 
     * @return double 
     */
    static double wrap_to_pi(double angle);
};

} // namespace control