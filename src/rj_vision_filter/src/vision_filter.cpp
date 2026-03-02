
#include "rj_vision_filter/vision_filter.hpp"

#include <rj_common/time.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_utils/logging_macros.hpp>

namespace vision_filter {

VisionFilterConfig VisionFilter::load_config() {
    VisionFilterConfig cfg;
    this->get_parameter("vision_loop_dt", cfg.vision_loop_dt);
    this->get_parameter("max_num_cameras", cfg.max_num_cameras);
    this->get_parameter("publish_hz", cfg.publish_hz);

    this->get_parameter("filter.health.init", cfg.filter_health.init);
    this->get_parameter("filter.health.inc", cfg.filter_health.inc);
    this->get_parameter("filter.health.dec", cfg.filter_health.dec);
    this->get_parameter("filter.health.max", cfg.filter_health.max);
    this->get_parameter("filter.health.min", cfg.filter_health.min);

    this->get_parameter("ball.init_covariance", cfg.ball.init_covariance);
    this->get_parameter("ball.process_noise", cfg.ball.process_noise);
    this->get_parameter("ball.observation_noise", cfg.ball.observation_noise);

    this->get_parameter("robot.init_covariance", cfg.robot.init_covariance);
    this->get_parameter("robot.process_noise", cfg.robot.process_noise);
    this->get_parameter("robot.observation_noise", cfg.robot.observation_noise);
    this->get_parameter("robot.orientation_scale", cfg.robot.orientation_scale);

    this->get_parameter("camera.mhkf_radius_cutoff", cfg.camera.mhkf_radius_cutoff);
    this->get_parameter("camera.use_mhkf", cfg.camera.use_mhkf);
    this->get_parameter("camera.max_num_kalman_balls", cfg.camera.max_num_kalman_balls);
    this->get_parameter("camera.max_num_kalman_robots", cfg.camera.max_num_kalman_robots);

    this->get_parameter("vision_filter.bounce.robot_body_lin_dampen",
                        cfg.bounce.robot_body_lin_dampen);
    this->get_parameter("vision_filter.bounce.robot_mouth_lin_dampen",
                        cfg.bounce.robot_mouth_lin_dampen);
    this->get_parameter("vision_filter.bounce.robot_body_angle_dampen",
                        cfg.bounce.robot_body_angle_dampen);
    this->get_parameter("vision_filter.bounce.robot_mouth_angle_dampen",
                        cfg.bounce.robot_mouth_angle_dampen);

    this->get_parameter("world_ball.ball_merger_power", cfg.world_ball.ball_merger_power);
    this->get_parameter("world_robot.robot_merger_power", cfg.world_robot.robot_merger_power);

    this->get_parameter("kalman_ball.max_time_outside_vision",
                        cfg.kalman_ball.max_time_outside_vision);
    this->get_parameter("kalman_robot.max_time_outside_vision",
                        cfg.kalman_robot.max_time_outside_vision);

    this->get_parameter("kick.detector.fast_acceleration_trigger",
                        cfg.kick_detector.fast_acceleration_trigger);
    this->get_parameter("kick.detector.fast_kick_hist_length",
                        cfg.kick_detector.fast_kick_hist_length);
    this->get_parameter("kick.detector.fast_kick_timeout", cfg.kick_detector.fast_kick_timeout);
    this->get_parameter("kick.detector.same_kick_timeout", cfg.kick_detector.same_kick_timeout);
    this->get_parameter("kick.detector.slow_any_robot_past_dist",
                        cfg.kick_detector.slow_any_robot_past_dist);
    this->get_parameter("kick.detector.slow_kick_hist_length",
                        cfg.kick_detector.slow_kick_hist_length);
    this->get_parameter("kick.detector.slow_kick_timeout", cfg.kick_detector.slow_kick_timeout);
    this->get_parameter("kick.detector.slow_max_kick_angle", cfg.kick_detector.slow_max_kick_angle);
    this->get_parameter("kick.detector.slow_min_ball_speed", cfg.kick_detector.slow_min_ball_speed);
    this->get_parameter("kick.detector.slow_one_robot_within_dist",
                        cfg.kick_detector.slow_one_robot_within_dist);
    this->get_parameter("kick.detector.slow_robot_dist_filter_cutoff",
                        cfg.kick_detector.slow_robot_dist_filter_cutoff);
    return cfg;
}

VisionFilter::VisionFilter(const rclcpp::NodeOptions& options)
    : rclcpp::Node{"vision_filter", options},
      config_client_{this},
      team_color_queue_{this, referee::topics::kTeamColorTopic},
      config_(load_config()),
      world_(config_) {
    // Create a timer that calls predict on all of the Kalman filters.
    const std::chrono::duration<double> predict_timer_period(config_.vision_loop_dt);
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
        world_.update_single_camera(RJ::now(), frame);
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
