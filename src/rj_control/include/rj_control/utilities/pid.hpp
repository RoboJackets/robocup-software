/**
 * @file pid.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <optional>
#include <cstdlib>
#include <algorithm>
#include <cmath>

namespace control {

class Pid {
public:
    /**
     * @brief Construct a new Pid object
     * 
     * @param maximum_output The maximum output from the pid controller
     * @param kp The proportional gain
     * @param ki The integral gain
     * @param kd The derivative gain
     */
    Pid(
        double maximum_output, // NOLINT(bugprone-easily-swappable-parameters)
        double kp, // NOLINT(readability-identifier-length)
        double ki, // NOLINT(readability-identifier-length)
        double kd // NOLINT(readability-identifier-length)
    ): kp_(kp), ki_(ki), kd_(kd), maximum_output_(maximum_output) {};

    Pid(Pid&& other) = default;
    Pid& operator=(Pid&&) = default;
    Pid(const Pid& other) = default;
    Pid& operator=(const Pid&) = default;


    virtual ~Pid() = default;

    /**
     * @brief Set the p limit
     * 
     * @param p_limit 
     */
    void set_p_limit(std::optional<double> p_limit) {
        p_limit_ = p_limit;
    }

    /**
     * @brief Get the p limit
     * 
     * @return std::optional<float> 
     */
    [[nodiscard]] std::optional<double> p_limit() const { return p_limit_; }

    /**
     * @brief Set the i limit
     * 
     * @param i_limit 
     */
    void set_i_limit(std::optional<double> i_limit) {
        i_limit_ = i_limit;
    }

    /**
     * @brief Get the i limit
     * 
     * @return std::optional<float> 
     */
    [[nodiscard]] std::optional<double> i_limit() const { return i_limit_; }

    /**
     * @brief Set the d limit 
     * 
     * @param d_limit 
     */
    void set_d_limit(std::optional<double> d_limit) {
        d_limit_ = d_limit;
    }

    /**
     * @brief Get the d limit
     * 
     * @return std::optional<float> 
     */
    [[nodiscard]] std::optional<double> d_limit() const { return d_limit_; }

    /**
     * @brief Set the limits of the p, i, and d contributions
     *
     * @note This value treats invalid limits as std::nullopts
     * 
     * @param p_limit The limit of the proportional contribution
     * @param i_limit The limit of the integral contribution
     * @param d_limit The limit of the derivate contribution
     */
    void set_limits(double p_limit, double i_limit, double d_limit) {
        p_limit_ = p_limit < 0.0 ? std::nullopt : std::optional(p_limit);
        i_limit_ = i_limit < 0.0 ? std::nullopt : std::optional(i_limit);
        d_limit_ = d_limit < 0.0 ? std::nullopt : std::optional(d_limit);
    }

    /**
     * @brief Set the proportional gain
     * 
     * @param kp 
     */
    // NOLINTNEXTLINE(readability-identifier-length)
    void set_kp(double kp) {
        kp_ = kp;
    }

    /**
     * @brief Get the proportional gain
     * 
     * @return float 
     */
    [[nodiscard]] double kp() const { return kp_; }

    /**
     * @brief Set the integral gain
     * 
     * @param ki 
     */
    // NOLINTNEXTLINE(readability-identifier-length)
    void set_ki(double ki) {
        ki_ = ki;
    }

    /**
     * @brief Get the integral gain
     * 
     * @return float 
     */
    [[nodiscard]] double ki() const { return ki_; }

    /**
     * @brief Set the derivative gain
     * 
     * @param kd 
     */
    // NOLINTNEXTLINE(readability-identifier-length)
    void set_kd(double kd) { 
        kd_ = kd;
    }

    /**
     * @brief Get the derivative gain
     * 
     * @return float 
     */
    [[nodiscard]] double kd() const { return kd_; }

    /**
     * @brief Set the maxium output
     * 
     * @param maximum_output 
     */
    void set_maxium_output(double maximum_output) {
        maximum_output_ = maximum_output;
    }

    /**
     * @brief Get the maximum output
     * 
     * @return double 
     */
    [[nodiscard]] double maximum_output() const { return maximum_output_; }

    /**
     * @brief Get the last error value
     * 
     * @return float 
     */
    [[nodiscard]] double last_error() const { return last_error_; }

    /**
     * @brief Perform a motion update on the setpoint
     * 
     * @param setpoint The desired position
     * @param measurement The measured position
     * @return float The command velocity
     */
    virtual double update(double setpoint, double measurement) = 0;

    /**
     * @brief Reset the integral term of the controller
     * 
     */
    void reset() {
        integral_term_ = 0.0;
        previous_measurement_ = std::nullopt;
        last_error_ = 0.0;
    };

protected:
    // The proportional gain of the controller
    double kp_;
    // The integral gain of the controller
    double ki_;
    // The derivative gain of the controller
    double kd_;

    // Limiter for the proportional gain `-p_limit <= P <= p_limit`
    std::optional<double> p_limit_ = std::nullopt;
    // Limiter for the integral gain `-i_limit <= I <= i_limit`
    std::optional<double> i_limit_ = std::nullopt;
    // Limiter for the derivative gain `d_limit <= D <= d_limit`
    std::optional<double> d_limit_ = std::nullopt;

    // The last calculated integral value
    double integral_term_ = 0.0;
    // The last measured value
    std::optional<double> previous_measurement_ = std::nullopt;
    // The error from the last measurement
    double last_error_ = 0.0;

    // The maximum output the controller can output
    double maximum_output_;
};

} // namespace control