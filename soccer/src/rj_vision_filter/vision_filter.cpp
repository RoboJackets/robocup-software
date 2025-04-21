
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_utils/logging_macros.hpp>
#include <rj_vision_filter/params.hpp>
#include <rj_vision_filter/vision_filter.hpp>

#include "world_state.hpp"

namespace vision_filter {
DEFINE_FLOAT64(kVisionFilterParamModule, publish_hz, 60.0,
               "The rate in Hz at which VisionFilter publishes ball and robot "
               "observations.")

VisionFilter::VisionFilter(const rclcpp::NodeOptions& options)
    : rclcpp::Node{"vision_filter", options},
      config_client_{this},
      team_color_queue_{this, referee::topics::kTeamColorTopic},
      param_provider_{this, kVisionFilterParamModule} {
    // Create a timer that calls predict on all of the Kalman filters.
    const std::chrono::duration<double> predict_timer_period(PARAM_vision_loop_dt);
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

    SPDLOG_INFO("Hello in vision filter");

    std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client =
    std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_param_provider");
    parameters_client->wait_for_service();

    SPDLOG_INFO("Hello in vision filter after creating parameters_client");


    auto parameters_future = parameters_client->get_parameters({
        "vision_filter.ball.init_covariance",
        "vision_filter.ball.observation_noise",
        "vision_filter.ball.process_noise",
        "vision_filter.camera.max_num_kalman_balls",
        "vision_filter.camera.max_num_kalman_robots",
        "vision_filter.camera.mhkf_radius_cutoff",
        "vision_filter.camera.use_mhkf",
        "vision_filter.filter.health.dec",
        "vision_filter.filter.health.inc",
        "vision_filter.filter.health.init",
        "vision_filter.filter.health.max",
        "vision_filter.filter.health.min",
        "vision_filter.kalman.ball.max_time_outside_vision",
        "vision_filter.kalman.robot.max_time_outside_vision",
        "vision_filter.kick.detector.fast_acceleration_trigger",
        "vision_filter.kick.detector.fast_kick_hist_length",
        "vision_filter.kick.detector.fast_kick_timeout",
        "vision_filter.kick.detector.same_kick_timeout",
        "vision_filter.kick.detector.slow_any_robot_past_dist",
        "vision_filter.kick.detector.slow_kick_hist_length",
        "vision_filter.kick.detector.slow_kick_timeout",
        "vision_filter.kick.detector.slow_max_kick_angle",
        "vision_filter.kick.detector.slow_min_ball_speed",
        "vision_filter.kick.detector.slow_one_robot_within_dist",
        "vision_filter.kick.detector.slow_robot_dist_filter_cutoff",
        "vision_filter.max_num_cameras",
        "vision_filter.publish_hz",
        "vision_filter.robot.init_covariance",
        "vision_filter.robot.observation_noise",
        "vision_filter.robot.orientation_scale",
        "vision_filter.robot.process_noise",
        "vision_filter.vision_loop_dt",
        "vision_filter.world.ball.ball_merger_power",
        "vision_filter.world.robot.robot_merger_power"
    });
    SPDLOG_INFO("Hello in vision filter after making parameters list");
    auto result = parameters_future.get();  // This will block until the result is available
    SPDLOG_INFO("Hello in vision filter after getting parameters");
    param_ball_init_covariance_ = result.at(0).as_double();
    param_ball_observation_noise_ = result.at(1).as_double();
    param_ball_process_noise_ = result.at(2).as_double();
    param_camera_max_num_kalman_balls_ = result.at(3).as_double();
    param_camera_max_num_kalman_robots_ = result.at(4).as_double();
    param_camera_mhkf_radius_cutoff_ = result.at(5).as_double();
    param_camera_use_mhkf_ = result.at(6).as_bool();
    param_filter_health_dec_ = result.at(7).as_int();
    param_filter_health_inc_ = result.at(8).as_int();
    param_filter_health_init_ = result.at(9).as_int();
    param_filter_health_max_ = result.at(10).as_int();
    param_filter_health_min_ = result.at(11).as_int();
    param_kalman_ball_max_time_outside_vision_ = result.at(12).as_double();
    param_kalman_robot_max_time_outside_vision_ = result.at(13).as_double();
    param_kick_detector_fast_acceleration_trigger_ = result.at(14).as_double();
    param_kick_detector_fast_kick_hist_length_ = result.at(15).as_double();
    param_kick_detector_fast_kick_timeout_ = result.at(16).as_double();
    param_kick_detector_same_kick_timeout_ = result.at(17).as_double();
    param_kick_detector_slow_any_robot_past_dist_ = result.at(18).as_double();
    param_kick_detector_slow_kick_hist_length_ = result.at(19).as_double();
    param_kick_detector_slow_kick_timeout_ = result.at(20).as_double();
    param_kick_detector_slow_max_kick_angle_ = result.at(21).as_double();
    param_kick_detector_slow_min_ball_speed_ = result.at(22).as_double();
    param_kick_detector_slow_one_robot_within_dist_ = result.at(23).as_double();
    param_kick_detector_slow_robot_dist_filter_cutoff_ = result.at(24).as_double();
    param_max_num_cameras_ = result.at(25).as_int();
    param_publish_hz_ = result.at(26).as_double();
    param_robot_init_covariance_ = result.at(27).as_double();
    param_robot_observation_noise_ = result.at(28).as_double();
    param_robot_orientation_scale_ = result.at(29).as_double();
    param_robot_process_noise_ = result.at(30).as_double();
    param_vision_loop_dt_ = result.at(31).as_double();
    param_world_ball_ball_merger_power_ = result.at(32).as_double();
    param_world_robot_robot_merger_power_ = result.at(33).as_double();
    SPDLOG_INFO("Hello in vision filter, world_robot_robot_merger_power_: {}", param_world_robot_robot_merger_power_);
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
