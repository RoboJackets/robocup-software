
#include "rj_vision_filter/vision_filter.hpp"

#include <rj_common/time.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_utils/logging_macros.hpp>

namespace vision_filter {
namespace {
/**
 * @brief Reads every vision filter parameter from the node's ROS2
 * parameters (falling back to the VisionFilterParams defaults for any
 * parameter that isn't set, e.g. when running without a launch file).
 */
VisionFilterParams load_vision_filter_params(rclcpp::Node* node) {
    VisionFilterParams params;

    node->get_parameter_or("vision_loop_dt", params.vision_loop_dt, params.vision_loop_dt);
    node->get_parameter_or("max_num_cameras", params.max_num_cameras, params.max_num_cameras);
    // TODO: publish_hz is unused (dead since before the rj_param_utils removal). Safe to remove.
    node->get_parameter_or("publish_hz", params.publish_hz, params.publish_hz);

    node->get_parameter_or("ball.init_covariance", params.ball.init_covariance,
                           params.ball.init_covariance);
    node->get_parameter_or("ball.process_noise", params.ball.process_noise,
                           params.ball.process_noise);
    node->get_parameter_or("ball.observation_noise", params.ball.observation_noise,
                           params.ball.observation_noise);

    node->get_parameter_or("robot.init_covariance", params.robot.init_covariance,
                           params.robot.init_covariance);
    node->get_parameter_or("robot.process_noise", params.robot.process_noise,
                           params.robot.process_noise);
    node->get_parameter_or("robot.observation_noise", params.robot.observation_noise,
                           params.robot.observation_noise);
    node->get_parameter_or("robot.orientation_scale", params.robot.orientation_scale,
                           params.robot.orientation_scale);

    node->get_parameter_or("camera.mhkf_radius_cutoff", params.camera.mhkf_radius_cutoff,
                           params.camera.mhkf_radius_cutoff);
    node->get_parameter_or("camera.use_mhkf", params.camera.use_mhkf, params.camera.use_mhkf);
    node->get_parameter_or("camera.max_num_kalman_balls", params.camera.max_num_kalman_balls,
                           params.camera.max_num_kalman_balls);
    node->get_parameter_or("camera.max_num_kalman_robots", params.camera.max_num_kalman_robots,
                           params.camera.max_num_kalman_robots);

    node->get_parameter_or("kalman_ball.max_time_outside_vision",
                           params.kalman_ball.max_time_outside_vision,
                           params.kalman_ball.max_time_outside_vision);
    node->get_parameter_or("kalman_robot.max_time_outside_vision",
                           params.kalman_robot.max_time_outside_vision,
                           params.kalman_robot.max_time_outside_vision);

    node->get_parameter_or("filter.health.init", params.filter_health.init,
                           params.filter_health.init);
    node->get_parameter_or("filter.health.inc", params.filter_health.inc,
                           params.filter_health.inc);
    node->get_parameter_or("filter.health.dec", params.filter_health.dec,
                           params.filter_health.dec);
    node->get_parameter_or("filter.health.max", params.filter_health.max,
                           params.filter_health.max);
    node->get_parameter_or("filter.health.min", params.filter_health.min,
                           params.filter_health.min);

    node->get_parameter_or("kick.detector.slow_kick_hist_length",
                           params.kick_detector.slow_kick_hist_length,
                           params.kick_detector.slow_kick_hist_length);
    node->get_parameter_or("kick.detector.fast_kick_hist_length",
                           params.kick_detector.fast_kick_hist_length,
                           params.kick_detector.fast_kick_hist_length);
    node->get_parameter_or("kick.detector.fast_kick_timeout",
                           params.kick_detector.fast_kick_timeout,
                           params.kick_detector.fast_kick_timeout);
    node->get_parameter_or("kick.detector.slow_kick_timeout",
                           params.kick_detector.slow_kick_timeout,
                           params.kick_detector.slow_kick_timeout);
    node->get_parameter_or("kick.detector.same_kick_timeout",
                           params.kick_detector.same_kick_timeout,
                           params.kick_detector.same_kick_timeout);
    node->get_parameter_or("kick.detector.fast_acceleration_trigger",
                           params.kick_detector.fast_acceleration_trigger,
                           params.kick_detector.fast_acceleration_trigger);
    // TODO: slow_robot_dist_filter_cutoff is unused (dead since before the rj_param_utils
    // removal). Safe to remove.
    node->get_parameter_or("kick.detector.slow_robot_dist_filter_cutoff",
                           params.kick_detector.slow_robot_dist_filter_cutoff,
                           params.kick_detector.slow_robot_dist_filter_cutoff);
    node->get_parameter_or("kick.detector.slow_one_robot_within_dist",
                           params.kick_detector.slow_one_robot_within_dist,
                           params.kick_detector.slow_one_robot_within_dist);
    node->get_parameter_or("kick.detector.slow_any_robot_past_dist",
                           params.kick_detector.slow_any_robot_past_dist,
                           params.kick_detector.slow_any_robot_past_dist);
    node->get_parameter_or("kick.detector.slow_min_ball_speed",
                           params.kick_detector.slow_min_ball_speed,
                           params.kick_detector.slow_min_ball_speed);
    node->get_parameter_or("kick.detector.slow_max_kick_angle",
                           params.kick_detector.slow_max_kick_angle,
                           params.kick_detector.slow_max_kick_angle);

    node->get_parameter_or("vision_filter.bounce.robot_body_lin_dampen",
                           params.bounce.robot_body_lin_dampen,
                           params.bounce.robot_body_lin_dampen);
    node->get_parameter_or("vision_filter.bounce.robot_mouth_lin_dampen",
                           params.bounce.robot_mouth_lin_dampen,
                           params.bounce.robot_mouth_lin_dampen);
    node->get_parameter_or("vision_filter.bounce.robot_body_angle_dampen",
                           params.bounce.robot_body_angle_dampen,
                           params.bounce.robot_body_angle_dampen);
    node->get_parameter_or("vision_filter.bounce.robot_mouth_angle_dampen",
                           params.bounce.robot_mouth_angle_dampen,
                           params.bounce.robot_mouth_angle_dampen);

    node->get_parameter_or("world_ball.ball_merger_power", params.world_ball.ball_merger_power,
                           params.world_ball.ball_merger_power);
    node->get_parameter_or("world_robot.robot_merger_power",
                           params.world_robot.robot_merger_power,
                           params.world_robot.robot_merger_power);

    return params;
}

using ParamSetter = std::function<void(const rclcpp::Parameter&)>;

/**
 * @brief Maps each ROS2 parameter name to a setter that writes the new value
 * into the given params struct, so a set_parameters_callback can keep
 * params live-updated without re-reading every parameter on every change.
 */
std::unordered_map<std::string, ParamSetter> build_param_setters(VisionFilterParams& params) {
    return {
        {"vision_loop_dt", [&params](const auto& p) { params.vision_loop_dt = p.as_double(); }},
        {"max_num_cameras", [&params](const auto& p) { params.max_num_cameras = p.as_int(); }},
        {"publish_hz", [&params](const auto& p) { params.publish_hz = p.as_double(); }},

        {"ball.init_covariance",
         [&params](const auto& p) { params.ball.init_covariance = p.as_double(); }},
        {"ball.process_noise",
         [&params](const auto& p) { params.ball.process_noise = p.as_double(); }},
        {"ball.observation_noise",
         [&params](const auto& p) { params.ball.observation_noise = p.as_double(); }},

        {"robot.init_covariance",
         [&params](const auto& p) { params.robot.init_covariance = p.as_double(); }},
        {"robot.process_noise",
         [&params](const auto& p) { params.robot.process_noise = p.as_double(); }},
        {"robot.observation_noise",
         [&params](const auto& p) { params.robot.observation_noise = p.as_double(); }},
        {"robot.orientation_scale",
         [&params](const auto& p) { params.robot.orientation_scale = p.as_double(); }},

        {"camera.mhkf_radius_cutoff",
         [&params](const auto& p) { params.camera.mhkf_radius_cutoff = p.as_double(); }},
        {"camera.use_mhkf", [&params](const auto& p) { params.camera.use_mhkf = p.as_bool(); }},
        {"camera.max_num_kalman_balls",
         [&params](const auto& p) { params.camera.max_num_kalman_balls = p.as_int(); }},
        {"camera.max_num_kalman_robots",
         [&params](const auto& p) { params.camera.max_num_kalman_robots = p.as_int(); }},

        {"kalman_ball.max_time_outside_vision",
         [&params](const auto& p) { params.kalman_ball.max_time_outside_vision = p.as_double(); }},
        {"kalman_robot.max_time_outside_vision",
         [&params](const auto& p) { params.kalman_robot.max_time_outside_vision = p.as_double(); }},

        {"filter.health.init", [&params](const auto& p) { params.filter_health.init = p.as_int(); }},
        {"filter.health.inc", [&params](const auto& p) { params.filter_health.inc = p.as_int(); }},
        {"filter.health.dec", [&params](const auto& p) { params.filter_health.dec = p.as_int(); }},
        {"filter.health.max", [&params](const auto& p) { params.filter_health.max = p.as_int(); }},
        {"filter.health.min", [&params](const auto& p) { params.filter_health.min = p.as_int(); }},

        {"kick.detector.slow_kick_hist_length",
         [&params](const auto& p) { params.kick_detector.slow_kick_hist_length = p.as_int(); }},
        {"kick.detector.fast_kick_hist_length",
         [&params](const auto& p) { params.kick_detector.fast_kick_hist_length = p.as_int(); }},
        {"kick.detector.fast_kick_timeout",
         [&params](const auto& p) { params.kick_detector.fast_kick_timeout = p.as_double(); }},
        {"kick.detector.slow_kick_timeout",
         [&params](const auto& p) { params.kick_detector.slow_kick_timeout = p.as_double(); }},
        {"kick.detector.same_kick_timeout",
         [&params](const auto& p) { params.kick_detector.same_kick_timeout = p.as_double(); }},
        {"kick.detector.fast_acceleration_trigger",
         [&params](const auto& p) {
             params.kick_detector.fast_acceleration_trigger = p.as_double();
         }},
        // TODO: slow_robot_dist_filter_cutoff is unused (dead since before the rj_param_utils
        // removal). Safe to remove.
        {"kick.detector.slow_robot_dist_filter_cutoff",
         [&params](const auto& p) {
             params.kick_detector.slow_robot_dist_filter_cutoff = p.as_double();
         }},
        {"kick.detector.slow_one_robot_within_dist",
         [&params](const auto& p) {
             params.kick_detector.slow_one_robot_within_dist = p.as_double();
         }},
        {"kick.detector.slow_any_robot_past_dist",
         [&params](const auto& p) {
             params.kick_detector.slow_any_robot_past_dist = p.as_double();
         }},
        {"kick.detector.slow_min_ball_speed",
         [&params](const auto& p) { params.kick_detector.slow_min_ball_speed = p.as_double(); }},
        {"kick.detector.slow_max_kick_angle",
         [&params](const auto& p) { params.kick_detector.slow_max_kick_angle = p.as_double(); }},

        {"vision_filter.bounce.robot_body_lin_dampen",
         [&params](const auto& p) { params.bounce.robot_body_lin_dampen = p.as_double(); }},
        {"vision_filter.bounce.robot_mouth_lin_dampen",
         [&params](const auto& p) { params.bounce.robot_mouth_lin_dampen = p.as_double(); }},
        {"vision_filter.bounce.robot_body_angle_dampen",
         [&params](const auto& p) { params.bounce.robot_body_angle_dampen = p.as_double(); }},
        {"vision_filter.bounce.robot_mouth_angle_dampen",
         [&params](const auto& p) { params.bounce.robot_mouth_angle_dampen = p.as_double(); }},

        {"world_ball.ball_merger_power",
         [&params](const auto& p) { params.world_ball.ball_merger_power = p.as_double(); }},
        {"world_robot.robot_merger_power",
         [&params](const auto& p) { params.world_robot.robot_merger_power = p.as_double(); }},
    };
}
}  // namespace

VisionFilter::VisionFilter(const rclcpp::NodeOptions& options)
    : rclcpp::Node{"vision_filter", options},
      params_{load_vision_filter_params(this)},
      world_{params_},
      config_client_{this},
      team_color_queue_{this, referee::topics::kTeamColorTopic},
      param_setters_{build_param_setters(params_)} {
    // Keep params_ live-updated whenever a parameter is changed (e.g. via
    // `ros2 param set`).
    param_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& changed_params) {
            for (const rclcpp::Parameter& param : changed_params) {
                auto it = param_setters_.find(param.get_name());
                if (it != param_setters_.end()) {
                    it->second(param);
                }
            }

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });

    // Create a timer that calls predict on all of the Kalman filters.
    const std::chrono::duration<double> predict_timer_period(params_.vision_loop_dt);
    auto publish_callback = [this]() { publish_state(); };
    publish_timer_ = create_wall_timer(predict_timer_period, publish_callback);

    // Create a subscriber for the DetectionFrameMsg
    constexpr int kQueueSize = 10;
    const auto callback = [this](DetectionFrameMsg::UniquePtr msg) {
        auto team_color = team_color_queue_.get();
        if (!config_client_.connected() || team_color == nullptr) {
            return;
        }

        const double current_team_angle = team_angle();
        const rj_geometry::TransformMatrix current_world_to_team = world_to_team();
        auto frame = CameraFrame(*msg, current_world_to_team, current_team_angle);
        world_.update_single_camera(RJ::now(), frame, params_);
    };
    detection_frame_sub_ = create_subscription<DetectionFrameMsg>(
        vision_receiver::topics::kDetectionFrameTopic, rclcpp::QoS(kQueueSize), callback);

    // Create publishers.
    world_state_pub_ = create_publisher<WorldStateMsg>(topics::kWorldStateTopic, 10);
}

VisionFilter::WorldStateMsg VisionFilter::build_world_state_msg(bool us_blue) const {
    return rj_msgs::build<WorldStateMsg>()
        .last_update_time(rj_convert::convert_to_ros(world_.last_update_time()))
        .their_robots(build_robot_state_msgs(!us_blue))
        .our_robots(build_robot_state_msgs(us_blue))
        .ball(build_ball_state_msg());
}

VisionFilter::BallStateMsg VisionFilter::build_ball_state_msg() const {
    const WorldBall& wb = world_.get_world_ball();

    BallStateMsg msg{};
    msg.stamp = rj_convert::convert_to_ros(wb.get_time());
    msg.position = rj_convert::convert_to_ros(wb.get_pos());
    msg.velocity = rj_convert::convert_to_ros(wb.get_vel());
    msg.visible = rj_convert::convert_to_ros(wb.get_is_valid());
    return msg;
}

std::vector<VisionFilter::RobotStateMsg> VisionFilter::build_robot_state_msgs(
    bool blue_team) const {
    const auto& robots = blue_team ? world_.get_robots_blue() : world_.get_robots_yellow();

    // Fill our robots
    std::vector<RobotStateMsg> robot_state_msgs(kNumShells);
    for (size_t i = 0; i < kNumShells; i++) {
        const WorldRobot& wr = robots.at(i);

        RobotState robot_state;
        robot_state.visible = wr.get_is_valid();

        if (wr.get_is_valid()) {
            robot_state.pose = rj_geometry::Pose(wr.get_pos(), wr.get_theta());
            robot_state.velocity = rj_geometry::Twist(wr.get_vel(), wr.get_omega());
            robot_state.timestamp = wr.get_time();
        }

        robot_state_msgs.at(i) = rj_convert::convert_to_ros(robot_state);
    }
    return robot_state_msgs;
}

void VisionFilter::publish_state() {
    std::shared_ptr<TeamColorMsg> team_color = team_color_queue_.get();
    if (team_color == nullptr) {
        EZ_WARN_THROTTLE(1000, "Returning because team_color is nullptr");
        return;
    }

    WorldStateMsg::UniquePtr msg = std::make_unique<WorldStateMsg>();
    *msg = build_world_state_msg(team_color->is_blue);
    world_state_pub_->publish(std::move(msg));
}

}  // namespace vision_filter
