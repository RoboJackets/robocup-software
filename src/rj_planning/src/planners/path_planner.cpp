#include "rj_planning/planners/path_planner.hpp"

namespace planning {

PathPlanner::~PathPlanner() {
    if (node_ != nullptr && replanner_params_callback_) {
        node_->remove_on_set_parameters_callback(replanner_params_callback_.get());
    }
    if (node_ != nullptr && rrt_params_callback_) {
        node_->remove_on_set_parameters_callback(rrt_params_callback_.get());
    }
    if (node_ != nullptr && intermediate_params_callback_) {
        node_->remove_on_set_parameters_callback(intermediate_params_callback_.get());
    }
}

PathPlanner::PathPlanner(rclcpp::Node *node, std::string name): node_{node}, name_(std::move(name)) {
        // Replanner params
    this->replanner_off_path_threshold =
        node_->get_parameter_or("replanner.off_path_threshold", 0.1);
    this->replanner_partial_replan_lead_time =
        node_->get_parameter_or("replanner.partial_replan_lead_time", 0.1);
    this->replanner_pos_change_threshold =
        node_->get_parameter_or("replanner.pos_change_threshold", 0.025);
    this->replanner_vel_change_threshold =
        node_->get_parameter_or("replanner.vel_change_threshold", 0.025);

    this->rrt_params_ = build_rrt_params();
    this->intermediate_params_ = build_intermediate_params();

    this->replanner_params_callback_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            if (param.get_name() == "replanner.off_path_threshold" ||
                param.get_name() == "replanner.partial_replan_lead_time" ||
                param.get_name() == "replanner.pos_change_threshold" ||
                param.get_name() == "replanner.vel_change_threshold") {
                if (!std::isfinite(param.as_double()) || param.as_double() < 0.0) {
                    result.successful = false;
                    result.reason =
                        fmt::format("{} must be finite and nonnegative", param.get_name());
                    return result;
                }
            }
        }

        for (const auto& param : params) {
            if (param.get_name() == "replanner.off_path_threshold") {
                replanner_off_path_threshold = param.as_double();
            } else if (param.get_name() == "replanner.partial_replan_lead_time") {
                replanner_partial_replan_lead_time = param.as_double();
            } else if (param.get_name() == "replanner.pos_change_threshold") {
                replanner_pos_change_threshold = param.as_double();
            } else if (param.get_name() == "replanner.vel_change_threshold") {
                replanner_vel_change_threshold = param.as_double();
            }
        }

        return result;
    });

    this->rrt_params_callback_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            if (param.get_name() == "rrt.goal_bias" ||
                param.get_name() == "rrt.step_size" ||
                param.get_name() == "rrt.waypoint_bias") {
                if (param.as_double() < 0.0) {
                    result.successful = false;
                    result.reason = fmt::format("{} must not be negative", param.get_name());
                    return result;
                }
            } else if (param.get_name() == "rrt.max_iterations" ||
                       param.get_name() == "rrt.min_iterations") {
                if (param.as_int() < 0) {
                    result.successful = false;
                    result.reason = fmt::format("{} must not be negative", param.get_name());
                    return result;
                }
            }
        }

        for (const auto& param : params) {
            if (param.get_name() == "rrt.enable_debug_drawing") {
                rrt_params_.enable_debug_drawing = param.as_bool();
            } else if (param.get_name() == "rrt.goal_bias") {
                rrt_params_.goal_bias = param.as_double();
            } else if (param.get_name() == "rrt.step_size") {
                rrt_params_.step_size = param.as_double();
            } else if (param.get_name() == "rrt.waypoint_bias") {
                rrt_params_.waypoint_bias = param.as_double();
            } else if (param.get_name() == "rrt.max_iterations") {
                rrt_params_.max_iterations = param.as_int();
            } else if (param.get_name() == "rrt.min_iterations") {
                rrt_params_.min_iterations = param.as_int();
            }
        }

        return result;
    });

    this->intermediate_params_callback_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto& param : params) {
                if (param.get_name() == "intermediate.min_scale" ||
                    param.get_name() == "intermediate.max_scale" ||
                    param.get_name() == "intermediate.min_angle" ||
                    param.get_name() == "intermediate.max_angle" ||
                    param.get_name() == "intermediate.step_size") {
                    if (!std::isfinite(param.as_double()) || param.as_double() < 0.0) {
                        result.successful = false;
                        result.reason =
                            fmt::format("{} must be finite and nonnegative", param.get_name());
                        return result;
                    }
                } else if (param.get_name() == "intermediate.num_intermediates" &&
                           param.as_int() < 0) {
                    result.successful = false;
                    result.reason = fmt::format("{} must not be negative", param.get_name());
                    return result;
                }
            }

            for (const auto& param : params) {
                if (param.get_name() == "intermediate.min_scale") {
                    intermediate_params_.min_scale = param.as_double();
                } else if (param.get_name() == "intermediate.max_scale") {
                    intermediate_params_.max_scale = param.as_double();
                } else if (param.get_name() == "intermediate.min_angle") {
                    intermediate_params_.min_angle = param.as_double();
                } else if (param.get_name() == "intermediate.max_angle") {
                    intermediate_params_.max_angle = param.as_double();
                } else if (param.get_name() == "intermediate.num_intermediates") {
                    intermediate_params_.num_intermediates = param.as_int();
                } else if (param.get_name() == "intermediate.step_size") {
                    intermediate_params_.step_size = param.as_double();
                }
            }

            return result;
        });


}

RrtParams PathPlanner::build_rrt_params() {
    RrtParams p;
    p.enable_debug_drawing =
        node_->get_parameter_or("rrt.enable_debug_drawing", false);
    p.goal_bias =
        node_->get_parameter_or("rrt.goal_bias", 0.3);
    p.step_size =
        node_->get_parameter_or("rrt.step_size", 0.15);
    p.waypoint_bias =
        node_->get_parameter_or("rrt.waypoint_bias", 0.5);
    p.max_iterations =
        node_->get_parameter_or("rrt.max_iterations", 250);
    p.min_iterations =
        node_->get_parameter_or("rrt.min_iterations", 0);
    return p;
}

IntermediateParams PathPlanner::build_intermediate_params() {
    IntermediateParams p;
    p.min_scale = node_->get_parameter_or("intermediate.min_scale", 0.5);
    p.max_scale = node_->get_parameter_or("intermediate.max_scale", 1.5);
    p.min_angle = node_->get_parameter_or("intermediate.min_angle", 20.0);
    p.max_angle = node_->get_parameter_or("intermediate.max_angle", 140.0);
    p.num_intermediates = node_->get_parameter_or("intermediate.num_intermediates", 5);
    p.step_size = node_->get_parameter_or("intermediate.step_size", 0.1);
    return p;
}



}