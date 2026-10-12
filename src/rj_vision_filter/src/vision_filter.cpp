#include "rj_vision_filter/vision_filter.hpp"

#include <rj_common/time.hpp>
#include <rj_common/topic_names.hpp>
#include <rj_common/world_state.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_common/ros_params.hpp>

namespace vision_filter {
namespace {
/// Declares every vision filter param (all required, no defaults) and reads it once.
std::shared_ptr<const VisionFilterParams> load_vision_filter_params(rclcpp::Node& node) {
    VisionFilterParams p{};
    auto require = [&node](auto& field, const std::string& name, const std::string& desc) {
        field = rj_utils::declare_required_param<std::decay_t<decltype(field)>>(node, name, desc);
    };

    require(p.vision_loop_dt, "vision_loop_dt", "1/freq of the vision loop (s)");
    require(p.max_num_cameras, "max_num_cameras", "Max number of cameras possible on the field");
    require(p.ball.init_covariance, "ball.init_covariance",
            "Initial covariance of the ball filter; controls how fast it converges");
    require(p.ball.process_noise, "ball.process_noise",
            "How quickly the ball filter reacts to changes in acceleration");
    require(p.ball.observation_noise, "ball.observation_noise",
            "How much the ball filter trusts camera measurements");
    require(p.robot.init_covariance, "robot.init_covariance",
            "Initial covariance of the robot filter; controls how fast it converges");
    require(p.robot.process_noise, "robot.process_noise",
            "How quickly the robot filter reacts to changes in acceleration");
    require(p.robot.observation_noise, "robot.observation_noise",
            "How much the robot filter trusts camera measurements");
    require(p.robot.orientation_scale, "robot.orientation_scale",
            "Scales covariance and noise to radians instead of meters");
    require(p.camera.mhkf_radius_cutoff, "camera.mhkf_radius_cutoff",
            "Radius (m) within which measurements associate with a Kalman object");
    require(p.camera.use_mhkf, "camera.use_mhkf", "Use MHKF (true) or AKF (false)");
    require(p.camera.max_num_kalman_balls, "camera.max_num_kalman_balls",
            "Max number of Kalman balls per camera");
    require(p.camera.max_num_kalman_robots, "camera.max_num_kalman_robots",
            "Max number of Kalman robots per robot id per camera");
    require(p.kalman_ball.max_time_outside_vision, "kalman_ball.max_time_outside_vision",
            "Seconds without a measurement before a ball filter is removed");
    require(p.kalman_robot.max_time_outside_vision, "kalman_robot.max_time_outside_vision",
            "Seconds without a measurement before a robot filter is removed");
    require(p.filter_health.init, "filter.health.init",
            "Initial health of a Kalman filter; must be between min and max");
    require(p.filter_health.inc, "filter.health.inc",
            "Health gained on each predict with a measurement");
    require(p.filter_health.dec, "filter.health.dec",
            "Health lost on each predict without a measurement");
    require(p.filter_health.max, "filter.health.max", "Max health of a Kalman filter");
    require(p.filter_health.min, "filter.health.min", "Min health of a Kalman filter");
    require(p.kick_detector.slow_kick_hist_length, "kick.detector.slow_kick_hist_length",
            "Frames stored for the slow kick detector");
    require(p.kick_detector.fast_kick_hist_length, "kick.detector.fast_kick_hist_length",
            "Frames stored for the fast kick detector");
    require(p.kick_detector.fast_kick_timeout, "kick.detector.fast_kick_timeout",
            "Only replace fast kick estimates after this many seconds");
    require(p.kick_detector.slow_kick_timeout, "kick.detector.slow_kick_timeout",
            "Only replace slow kick estimates after this many seconds");
    require(p.kick_detector.same_kick_timeout, "kick.detector.same_kick_timeout",
            "Replace a fast kick estimate with a slow one only within this many seconds");
    require(p.kick_detector.fast_acceleration_trigger, "kick.detector.fast_acceleration_trigger",
            "Acceleration (m/s^2) that triggers the fast kick detector");
    require(p.kick_detector.slow_one_robot_within_dist, "kick.detector.slow_one_robot_within_dist",
            "Only one ball measurement may be within this distance (m) of the robot");
    require(p.kick_detector.slow_any_robot_past_dist, "kick.detector.slow_any_robot_past_dist",
            "At least one ball measurement must be past this distance (m) of the robot");
    require(p.kick_detector.slow_min_ball_speed, "kick.detector.slow_min_ball_speed",
            "Minimum ball speed (m/s) for the slow kick detector");
    require(p.kick_detector.slow_max_kick_angle, "kick.detector.slow_max_kick_angle",
            "Max angle (rad) between ball velocity and robot heading");
    require(p.bounce.robot_body_angle_dampen, "vision_filter.bounce.robot_body_angle_dampen",
            "Fraction of the reflect angle kept when bouncing off the shell");
    require(p.bounce.robot_mouth_angle_dampen, "vision_filter.bounce.robot_mouth_angle_dampen",
            "Fraction of the reflect angle kept when bouncing off the mouth");
    require(p.world_ball.ball_merger_power, "world_ball.ball_merger_power",
            "Exponent applied to filter health when merging balls across cameras");
    require(p.world_robot.robot_merger_power, "world_robot.robot_merger_power",
            "Exponent applied to filter health when merging robots across cameras");
    return std::make_shared<const VisionFilterParams>(p);
}
}  // namespace

VisionFilter::VisionFilter(const rclcpp::NodeOptions& options)
    : rclcpp::Node{"vision_filter", options},
      // Read before constructing world_: its cameras read params on construction.
      params_{load_vision_filter_params(*this)},
      world_{params_},
      config_client_{this} {
    // Create a timer that calls predict on all of the Kalman filters.
    const std::chrono::duration<double> predict_timer_period(params_->vision_loop_dt);
    auto publish_callback = [this]() { publish_state(); };
    publish_timer_ = create_wall_timer(predict_timer_period, publish_callback);

    team_color_sub_ = create_subscription<TeamColorMsg>(
        referee::topics::kTeamColorTopic, rclcpp::QoS(1),
        [this](TeamColorMsg::UniquePtr msg) { is_blue_ = msg->is_blue; });

    // Create a subscriber for the DetectionFrameMsg
    constexpr int kQueueSize = 10;
    const auto callback = [this](DetectionFrameMsg::UniquePtr msg) {
        if (!config_client_.connected()) {
            return;
        }

        const double current_team_angle = team_angle();
        const rj_geometry::TransformMatrix current_world_to_team = world_to_team();
        auto frame = CameraFrame(*msg, current_world_to_team, current_team_angle);
        world_.update_single_camera(RJ::now(), frame);
    };
    detection_frame_sub_ = create_subscription<DetectionFrameMsg>(
        vision_receiver::topics::kDetectionFrameTopic, rclcpp::QoS(kQueueSize), callback);

    // Create world state publisher
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
    WorldStateMsg::UniquePtr msg = std::make_unique<WorldStateMsg>(build_world_state_msg(is_blue_));
    world_state_pub_->publish(std::move(msg));
}

}  // namespace vision_filter
