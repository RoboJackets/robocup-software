#include <iostream>

#include <robot.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_utils/logging.hpp>
#include <rj_vision_filter/vision_filter.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {
DEFINE_FLOAT64(kVisionFilterParamModule, publish_hz, 120.0,
               "The rate in Hz at which VisionFilter publishes ball and robot "
               "observations.")

VisionFilter::VisionFilter(const rclcpp::NodeOptions& options)
    : rclcpp::Node{"vision_filter", options},
      config_client_{this},
      team_color_queue_{this, referee::topics::kTeamColorPub,
                        rj_msgs::build<TeamColorMsg>().is_blue(true)},
      param_provider_{this, kVisionFilterParamModule} {
    // Create a timer that calls predict on all of the Kalman filters.
    const std::chrono::duration<double> predict_timer_period(PARAM_vision_loop_dt);
    auto predict_callback = [this]() { predict_states(); };
    predict_timer_ = create_wall_timer(predict_timer_period, predict_callback);

    // Create a subscriber for the DetectionFrameMsg
    constexpr int kQueueSize = 10;
    const auto callback = [this](DetectionFrameMsg::UniquePtr msg) {
        detection_frame_queue_.push(std::move(msg));
    };
    detection_frame_sub_ = create_subscription<DetectionFrameMsg>(
        vision_receiver::topics::kDetectionFramePub, rclcpp::QoS(kQueueSize), callback);

    // Create publishers.
    world_state_pub_ = create_publisher<WorldStateMsg>(topics::kWorldStatePub, 10);
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
            robot_state.pose = Geometry2d::Pose(wr.get_pos(), wr.get_theta());
            robot_state.velocity = Geometry2d::Twist(wr.get_vel(), wr.get_omega());
            robot_state.timestamp = wr.get_time();
        }

        robot_state_msgs.at(i) = rj_convert::convert_to_ros(robot_state);
    }
    return robot_state_msgs;
}

void VisionFilter::publish_state() {
    std::shared_ptr<TeamColorMsg> team_color = team_color_queue_.get();
    if (team_color == nullptr) {
        EZ_WARN_STREAM("Returning because team_color is nullptr");
        return;
    }

    WorldStateMsg::UniquePtr msg = std::make_unique<WorldStateMsg>();
    *msg = build_world_state_msg(team_color->is_blue);
    world_state_pub_->publish(std::move(msg));
}

void VisionFilter::get_frames() {
    std::vector<DetectionFrameMsg::UniquePtr> raw_frames = detection_frame_queue_.get_all();
    if (raw_frames.empty()) {
        return;
    }

    const double current_team_angle = team_angle();
    const Geometry2d::TransformMatrix current_world_to_team = world_to_team();
    for (const DetectionFrameMsg::UniquePtr& msg : raw_frames) {
        frame_buffer_.emplace_back(*msg, current_world_to_team, current_team_angle);
    }
}
void VisionFilter::predict_states() {
    const RJ::Time start = RJ::now();

    // Perform the updates on the Kalman Filters.
    predict_states_impl();

    // Check that predict_states runs fast enough, otherwise print a warning.
    const RJ::Seconds predict_time = RJ::now() - start;
    const RJ::Seconds diff_duration = RJ::Seconds(PARAM_vision_loop_dt) - predict_time;

    if (diff_duration < RJ::Seconds{0}) {
        constexpr int kWarningThrottleMS = 1000;
        EZ_WARN_STREAM_THROTTLE(kWarningThrottleMS,
                                "Predict is not called fast enough. Iteration took "
                                    << diff_duration.count() << " seconds, should be "
                                    << PARAM_vision_loop_dt << ".");
    }
}

void VisionFilter::predict_states_impl() {
    // Do update with whatever is in frame buffer
    get_frames();

    if (!frame_buffer_.empty()) {
        world_.update_with_camera_frame(RJ::now(), frame_buffer_);
        frame_buffer_.clear();
    } else {
        world_.update_without_camera_frame(RJ::now());
    }

    // Publish the updated world state at the end.
    publish_state();
}
}  // namespace vision_filter
