#pragma once

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_global_param_provider.hpp>

constexpr auto kGlobalParamModule = "soccer";
constexpr auto kGlobalParamServerNode = "parameter_blackboard";

DECLARE_BOOL(kGlobalParamModule, use_sim_time)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::physics, ball_decay_constant)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_speed)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_acceleration)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::robot, min_kick_speed)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_kick_speed)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::robot, min_chip_speed)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_chip_speed)
DECLARE_NS_INT64(kGlobalParamModule, soccer::robot, min_safe_kick_power)
DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::robot, robot_radius)

void start_global_param_provider(rclcpp::Node* node, const std::string& global_param_server);

void start_global_param_provider(const std::string& program_id,
                                 const std::string& global_param_server);

/**
 * Call this function to guarantee that all of the static variables will be initialized.
 * This is a horrible hack.
 */
void global_params_server_dummy_function();
