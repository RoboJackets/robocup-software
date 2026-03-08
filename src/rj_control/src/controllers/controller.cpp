#include "rj_control/controllers/controller.hpp"

namespace control {

Controller::Controller(int robot_id, rclcpp::Node::SharedPtr control_node, std::string name)
    : robot_id_(robot_id),
      control_node_(std::move(control_node)),
      x_controller_({1.0, 0.0, 0.0, 0.0}),
      y_controller_({1.0, 0.0, 0.0, 0.0}),
      w_controller_({1.0, 0.0, 0.0, 0.0})
{
    // Create error publishers
    error_x_pub_ = control_node_->create_publisher<std_msgs::msg::Float64>(
        fmt::format("{}/pose_error_x", name), rclcpp::QoS(1).best_effort()
    );
    error_y_pub_ = control_node_->create_publisher<std_msgs::msg::Float64>(
        fmt::format("{}/pose_error_y", name), rclcpp::QoS(1).best_effort()
    );
    error_heading_pub_ = control_node_->create_publisher<std_msgs::msg::Float64>(
        fmt::format("{}/pose_error_theta", name), rclcpp::QoS(1).best_effort()
    );

    // Create a subscription to reset the pid error
    reset_sub_ = control_node_->create_subscription<std_msgs::msg::Bool>(
        fmt::format("{}/reset", name), rclcpp::QoS(1).best_effort(),
        //NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                reset();
            }
        }
    );

    load_pid_params();

    param_cb_handle_ = control_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        update_params(params);

        return result;
    });
}

void Controller::load_pid_params() {
    double kp_trans = control_node_->get_parameter("pid.translational.p.gain").as_double();
    double ki_trans = control_node_->get_parameter("pid.translational.i.gain").as_double();
    double kd_trans = control_node_->get_parameter("pid.translational.d.gain").as_double();
    double max_output_trans = control_node_->get_parameter("pid.translational.max_output").as_double();
    x_controller_ = TranslationalPid(max_output_trans, kp_trans, ki_trans, kd_trans);
    y_controller_ = TranslationalPid(max_output_trans, kp_trans, ki_trans, kd_trans);
    double p_limit_trans = control_node_->get_parameter("pid.translational.p.limit").as_double();
    double i_limit_trans = control_node_->get_parameter("pid.translational.i.limit").as_double();
    double d_limit_trans = control_node_->get_parameter("pid.translational.d.limit").as_double();
    x_controller_.set_limits(p_limit_trans, i_limit_trans, d_limit_trans);
    y_controller_.set_limits(p_limit_trans, i_limit_trans, d_limit_trans);

    double kp_rot = control_node_->get_parameter("pid.rotational.p.gain").as_double();
    double ki_rot = control_node_->get_parameter("pid.rotational.i.gain").as_double();
    double kd_rot = control_node_->get_parameter("pid.rotational.d.gain").as_double();
    double max_output_rot = control_node_->get_parameter("pid.rotational.max_output").as_double();
    w_controller_ = RotationalPid(max_output_rot, kp_rot, ki_rot, kd_rot);
    double p_limit_rot = control_node_->get_parameter("pid.rotational.p.limit").as_double();
    double i_limit_rot = control_node_->get_parameter("pid.rotational.i.limit").as_double();
    double d_limit_rot = control_node_->get_parameter("pid.rotational.d.limit").as_double();
    w_controller_.set_limits(p_limit_rot, i_limit_rot, d_limit_rot);
}

//NOLINTNEXTLINE(readability-function-cognitive-complexity)
void Controller::update_params(const std::vector<rclcpp::Parameter>& params) {
    for (const auto& param : params) {
        if (param.get_name() == "pid.translational.p.gain") {
            SPDLOG_INFO("Setting kp to {}", param.as_double());
            x_controller_.set_kp(param.as_double());
            y_controller_.set_kp(param.as_double());
        } else if (param.get_name() == "pid.translational.i.gain") {
            x_controller_.set_ki(param.as_double());
            y_controller_.set_ki(param.as_double());
        } else if (param.get_name() == "pid.translational.d.gain") {
            x_controller_.set_kd(param.as_double());
            y_controller_.set_kd(param.as_double());
        } else if (param.get_name() == "pid.translational.max_output") {
            x_controller_.set_maxium_output(param.as_double());
            y_controller_.set_maxium_output(param.as_double());
        } else if (param.get_name() == "pid.translational.p.limit") {
            double value = param.as_double();
            if (value < 0.0) {
                x_controller_.set_p_limit(std::nullopt);
                y_controller_.set_p_limit(std::nullopt);
            } else {
                x_controller_.set_p_limit(param.as_double());
                y_controller_.set_p_limit(param.as_double());
            }
        } else if (param.get_name() == "pid.translational.i.limit") {
            double value = param.as_double();
            if (value < 0.0) {
                x_controller_.set_i_limit(std::nullopt);
                y_controller_.set_i_limit(std::nullopt);
            } else {
                x_controller_.set_i_limit(param.as_double());
                y_controller_.set_i_limit(param.as_double());
            }
        } else if (param.get_name() == "pid.translational.d.limit") {
            double value = param.as_double();
            if (value < 0.0) {
                x_controller_.set_d_limit(std::nullopt);
                y_controller_.set_d_limit(std::nullopt);
            } else {
                x_controller_.set_d_limit(param.as_double());
                y_controller_.set_d_limit(param.as_double());
            }
        } else if (param.get_name() == "pid.rotational.p.gain") {
            w_controller_.set_kp(param.as_double());
        } else if (param.get_name() == "pid.rotational.i.gain") {
            w_controller_.set_ki(param.as_double());
        } else if (param.get_name() == "pid.rotational.d.gain") {
            w_controller_.set_kd(param.as_double());
        } else if (param.get_name() == "pid.rotational.max_output") {
            w_controller_.set_maxium_output(param.as_double());
        } else if (param.get_name() == "pid.rotational.p.limit") {
            double value = param.as_double();
            if (value < 0.0) {
                w_controller_.set_p_limit(std::nullopt);
            } else {
                w_controller_.set_p_limit(param.as_double());
            }
        } else if (param.get_name() == "pid.rotational.i.limit") {
            double value = param.as_double();
            if (value < 0.0) {
                w_controller_.set_i_limit(std::nullopt);
            } else {
                w_controller_.set_i_limit(param.as_double());
            }
        } else if (param.get_name() == "pid.rotational.d.limit") {
            double value = param.as_double();
            if (value < 0.0) {
                w_controller_.set_d_limit(std::nullopt);
            } else {
                w_controller_.set_d_limit(param.as_double());
            }
        }
    }
}

void Controller::publish_errors() {
    if (error_x_pub_->get_subscription_count() > 0) {
        std_msgs::msg::Float64 msg;
        msg.data = x_controller_.last_error();
        error_x_pub_->publish(msg);
    }

    if (error_y_pub_->get_subscription_count() > 0) {
        std_msgs::msg::Float64 msg;
        msg.data = y_controller_.last_error();
        error_y_pub_->publish(msg);
    }

    if (error_heading_pub_->get_subscription_count() > 0) {
        std_msgs::msg::Float64 msg;
        msg.data = w_controller_.last_error();
        error_heading_pub_->publish(msg);
    }
}

void Controller::reset() {
    x_controller_.reset();
    y_controller_.reset();
    w_controller_.reset();
}

} // namespace control