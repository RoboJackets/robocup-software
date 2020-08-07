#include <iostream>

#include <Robot.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_utils/logging.hpp>
#include <rj_vision_filter/VisionFilter.hpp>
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
    auto predict_callback = [this]() { PredictStates(); };
    predict_timer_ = create_wall_timer(predict_timer_period, predict_callback);

    // Create a subscriber for the DetectionFrameMsg
    constexpr int kQueueSize = 10;
    const auto callback = [this](DetectionFrameMsg::UniquePtr msg) {
        detection_frame_queue_.Push(std::move(msg));
    };
    detection_frame_sub_ = create_subscription<DetectionFrameMsg>(
        vision_receiver::topics::kDetectionFramePub, rclcpp::QoS(kQueueSize), callback);

    // Create publishers.
    world_state_pub_ = create_publisher<WorldStateMsg>(topics::kWorldStatePub, 10);
}

VisionFilter::WorldStateMsg VisionFilter::BuildWorldStateMsg(bool us_blue) const {
    return rj_msgs::build<WorldStateMsg>()
        .last_update_time(rj_convert::convert_to_ros(world.last_update_time()))
        .their_robots(BuildRobotStateMsgs(!us_blue))
        .our_robots(BuildRobotStateMsgs(us_blue))
        .ball(BuildBallStateMsg());
}

VisionFilter::BallStateMsg VisionFilter::BuildBallStateMsg() const {
    const WorldBall& wb = world.getWorldBall();

    BallStateMsg msg{};
    msg.stamp = rj_convert::convert_to_ros(wb.getTime());
    msg.position = rj_convert::convert_to_ros(wb.getPos());
    msg.velocity = rj_convert::convert_to_ros(wb.getVel());
    msg.visible = rj_convert::convert_to_ros(wb.getIsValid());
    return msg;
}

std::vector<VisionFilter::RobotStateMsg> VisionFilter::BuildRobotStateMsgs(bool blue_team) const {
    const auto& robots = blue_team ? world.getRobotsBlue() : world.getRobotsYellow();

    // Fill our robots
    std::vector<RobotStateMsg> robot_state_msgs(Num_Shells);
    for (size_t i = 0; i < Num_Shells; i++) {
        const WorldRobot& wr = robots.at(i);

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();

        if (wr.getIsValid()) {
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity = Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        robot_state_msgs.at(i) = rj_convert::convert_to_ros(robot_state);
    }
    return robot_state_msgs;
}

void VisionFilter::PublishState() {
    std::shared_ptr<TeamColorMsg> team_color = team_color_queue_.Get();
    if (team_color == nullptr) {
        EZ_WARN_STREAM("Returning because team_color is nullptr");
        return;
    }

    WorldStateMsg::UniquePtr msg = std::make_unique<WorldStateMsg>();
    *msg = BuildWorldStateMsg(team_color->is_blue);
    world_state_pub_->publish(std::move(msg));
}

void VisionFilter::GetFrames() {
    std::vector<DetectionFrameMsg::UniquePtr> raw_frames = detection_frame_queue_.GetAll();
    if (raw_frames.empty()) {
        return;
    }

    const double team_angle = TeamAngle();
    const Geometry2d::TransformMatrix world_to_team = WorldToTeam();
    for (const DetectionFrameMsg::UniquePtr& msg : raw_frames) {
        frameBuffer.emplace_back(*msg, world_to_team, team_angle);
    }
}
void VisionFilter::PredictStates() {
    const RJ::Time start = RJ::now();

    // Perform the updates on the Kalman Filters.
    PredictStatesImpl();

    // Check that PredictStates runs fast enough, otherwise print a warning.
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

void VisionFilter::PredictStatesImpl() {
    // Do update with whatever is in frame buffer
    GetFrames();

    if (!frameBuffer.empty()) {
        world.updateWithCameraFrame(RJ::now(), frameBuffer);
        frameBuffer.clear();
    } else {
        world.updateWithoutCameraFrame(RJ::now());
    }

    // Publish the updated world state at the end.
    PublishState();
}
}  // namespace vision_filter
