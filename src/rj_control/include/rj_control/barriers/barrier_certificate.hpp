/**
 * @file barrier_certificate.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Abstract barrier certificate class.  In short, a barrier certificate defines certain
 * regions as safe (i.e. rergions without collisions) and other regions as unsafe.  It then
 * uses convex optimization to ensure that all control inputs result in staying in the safe
 * set.
 * @version 0.1
 * @date 2025-12-23
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

#include <rj_geometry/pose.hpp>
#include <rj_common/world_state.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/field_dimensions.hpp>

namespace control {

class BarrierCertificate {
public:
    /**
     * @brief Construct a new Barrier Certificate object
     * 
     */
    BarrierCertificate() = default;

    /**
     * @brief Construct a new Barrier Certificate
     * 
     * @param robot_id The id of the robot being controlled
     * @param control_gain The control gain for the barrier
     * @param dt The time between calls to the barrier
     * @param goalie Is the barrier controlling the goalie
     */
    //NOLINTNEXTLINE(bugprone-easily-swappable-parameters, readability-identifier-length)
    BarrierCertificate(int robot_id, double control_gain, double dt, bool goalie = false)
        : robot_id_(robot_id),
          control_gain_(control_gain),
          dt_(dt),
          goalie_(goalie) {}

    /**
     * @brief Destroy the Barrier Certificate object
     * 
     */
    virtual ~BarrierCertificate() = default;

    /**
     * @brief Default move constructor for the Barrier Certificate
     * 
     * @param other 
     */
    BarrierCertificate(BarrierCertificate&& other) = default;

    /**
     * @brief Default move assignment
     * 
     * @return BarrierCertificate& 
     */
    BarrierCertificate& operator=(BarrierCertificate&&) = default;

    /**
     * @brief Default Copy Constructor fo the controller
     * 
     * @param other 
     */
    BarrierCertificate(const BarrierCertificate& other) = default;

    /**
     * @brief Default copy assignment
     * 
     * @return BarrierCertificate& 
     */
    BarrierCertificate& operator=(const BarrierCertificate&) = default;

    /**
     * @brief Apply the barrier certificate to the control input taking into account
     * the world state
     * 
     * @param world_state The current world state
     * @param play_state The current play state
     * @param field_dimensions The current field dimensions
     * @param control_input The control input for the robot
     * @param avoid_ball Should we treat the ball as an obstacle
     * @return rj_geometry::Twist The updated control input for the robot
     */
    virtual rj_geometry::Twist apply(
        const WorldState& world_state,
        const PlayState& play_state,
        const FieldDimensions& field_dimensions,
        const rj_geometry::Twist& control_input,
        bool avoid_ball = false
    ) = 0;

    /**
     * @brief Set the robot id of the barrier
     * 
     * @param robot_id 
     */
    void set_robot_id(int robot_id) {
        if (robot_id != robot_id_) {
            robot_id_ = robot_id;
        }
    }

    /**
     * @brief Get the robot id of the barrier
     * 
     * @return int 
     */
    [[nodiscard]] int robot_id() const {
        return robot_id_;
    }

    /**
     * @brief Set the control gain of the barrier certificate
     * 
     * @param control_gain The new control gain
     */
    void set_control_gain(double control_gain) {
        control_gain_ = control_gain;
    }

    /**
     * @brief Get the control gain of the barrier certificate
     * 
     * @return double 
     */
    [[nodiscard]] double control_gain() const {
        return control_gain_;
    }

    /**
     * @brief Set the time delta between barrier calls
     * 
     * @param dt 
     */
    void set_dt(double dt) { //NOLINT(readability-identifier-length)
        dt_ = dt;
    }

    /**
     * @brief Get the current time delta between barrier calls
     * 
     * @return double 
     */
    [[nodiscard]] double dt() const {
        return dt_;
    }

    /**
     * @brief Set whether this barrier is attached to the goalie
     * 
     * @param goalie 
     */
    void set_goalie(bool goalie) {
        goalie_ = goalie;
    }

    /**
     * @brief Is this barrier attached to the goalie
     * 
     * @return true 
     * @return false 
     */
    [[nodiscard]] bool goalie() const {
        return goalie_;
    }

protected:
    // The robot id the barrier is controlling
    int robot_id_ = 0.0;
    // The control gain of the barrier
    double control_gain_ = 1.0;
    // The time between calls to the barrier
    double dt_ = 1.0 / 2.0;
    // Is the controlled robot the goalie
    bool goalie_ = false;
};

} // namespace control