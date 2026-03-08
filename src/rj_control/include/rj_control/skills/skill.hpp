/**
 * @file state.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Helper composable for controllers that contains the basic building blocks
 * of specific collective actions
 * @version 0.1
 * @date 2026-01-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <rj_common/world_state.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rj_control_extensions/control_command.hpp>

#include "rj_control/utilities/pid.hpp"

namespace control {

/**
 * @brief A specific state of a controller
 * 
 */

template <typename Command, typename Tolerance> class Skill {
public:
    /**
     * @brief Construct a new State 
     * 
     */
    Skill() = default;

    /**
     * @brief Destroy the State object
     * 
     */
    virtual ~Skill() = default;

    /**
     * @brief Default move constructor for the state
     * 
     * @param other
     */
    Skill(Skill&& other) noexcept = default;

    /**
     * @brief Default move assignment
     */
    Skill& operator=(Skill&&) noexcept = default;

    /**
     * @brief Default copy constructor for the state
     * 
     * @param other
     */
    Skill(const Skill& other) = default;

    /**
     * @brief Default copy assignment
     */
    Skill& operator=(const Skill&) = default;

    /**
     * @brief Get the control command resulting in applying the state
     * 
     * @param robot_id The id of the robot to control
     * @param world_state The current state of the world
     * @param command The data necessary for the state to operate
     * @param x_controller The pid controller for the x-direction
     * @param y_controller The pid controller for the y-direction
     * @param w_controller The pid controller for rotation
     * @return ControlCommand The control command resulting in applying the state
     */
    virtual ControlCommand update(
        int robot_id,
        const WorldState& world_state,
        const Command& command,
        Pid& x_controller,
        Pid& y_controller,
        Pid& w_controller
    ) = 0;

    /**
     * @brief Determine whether the current state is finished
     * 
     * @param robot_id The id of the robot to control
     * @param world_state The current state of the world
     * @param command The command to execute
     * @param tolerance The tolerance criteria
     * @return true The state is finished
     * @return false The state is not finished
     */
    virtual bool complete(
        int robot_id,
        const WorldState& world_state,
        const Command& command,
        const Tolerance& tolerance
    ) = 0;
};

} // namespace control