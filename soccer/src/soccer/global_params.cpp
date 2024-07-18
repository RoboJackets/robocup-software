#include <global_params.hpp>

DEFINE_BOOL(kGlobalParamModule, use_sim_time, false, "Use sim time.")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::physics, ball_decay_constant, 0.180,
                  "Ball decay constant.")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_speed, 8.0, "Maximum robot speed, m/s.")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_acceleration, 5.0,
                  "Maximum robot acceleration, m/s^2")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_dribbler_speed, 2000.0, "Maximum robot dribbler speed, revolutions per minute")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, min_kick_speed, 1.0,
                  "Minimum kick speed extrapolated to speed at kick power 0, m/s")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_kick_speed, 7.0,
                  "Maximum kick speed at kick power 255, m/s")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, chip_angle, 40, "Chip angle, degrees")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, min_chip_speed, 0.5,
                  "Minimum chip speed extrapolated to speed at kick power 0, m/s")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, max_chip_speed, 7.0,
                  "Maximum chip speed at kick power 255, m/s")
DEFINE_NS_FLOAT64(kGlobalParamModule, soccer::robot, robot_radius, 0.09, "Robot radius, m")
DEFINE_NS_INT64(kGlobalParamModule, soccer::robot, min_safe_kick_power, 64,
                "Minimum safe discharge power for the kicker (0-255)");

void start_global_param_provider(rclcpp::Node* node, const std::string& global_param_server) {
    static std::unique_ptr<params::ROS2GlobalParamProvider> provider;
    provider = std::make_unique<params::ROS2GlobalParamProvider>(node, kGlobalParamModule,
                                                                 global_param_server);
}

void start_global_param_provider(const std::string& program_id,
                                 const std::string& global_param_server) {
    static std::unique_ptr<std::thread> gpp_thread;
    static rclcpp::Node::SharedPtr node;
    node = std::make_shared<rclcpp::Node>(fmt::format("{}_global_param_receiver", program_id));

    start_global_param_provider(node.get(), global_param_server);

    gpp_thread = std::make_unique<std::thread>([]() { rclcpp::spin(node); });
}

void global_params_server_dummy_function() {}
