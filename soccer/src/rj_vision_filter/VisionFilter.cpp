#include <Robot.hpp>
#include <iostream>
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
      param_provider_{this, kVisionFilterParamModule} {
    // Create a timer that publishes the current state of the ball + robots.
    publish_timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const std::chrono::duration<double> publish_timer_period(1 /
                                                             PARAM_publish_hz);
    publish_timer_ = create_wall_timer(
        publish_timer_period, [this]() { PublishState(); },
        publish_timer_callback_group_);

    // Create a timer that calls predict on all of the Kalman filters.
    predict_timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    const std::chrono::duration<double> predict_timer_period(
        PARAM_vision_loop_dt);
    auto predict_callback = [this]() { PredictStates(); };
    predict_timer_ = create_wall_timer(publish_timer_period, predict_callback,
                                       predict_timer_callback_group_);

    // Create a subscriber for the DetectionFrameMsg
    update_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions detection_frame_sub_opts{};
    detection_frame_sub_opts.callback_group = update_callback_group_;
    constexpr int kQueueSize = 10;
    const auto callback = [this](DetectionFrameMsg::UniquePtr msg) {
        detection_frame_queue_.Push(std::move(msg));
    };
    detection_frame_sub_ = create_subscription<DetectionFrameMsg>(
        vision_receiver::topics::kDetectionFramePub, rclcpp::QoS(kQueueSize),
        callback, detection_frame_sub_opts);

    // Create publishers.
    world_state_pub_ =
        create_publisher<WorldStateMsg>(topics::kWorldStatePub, 10);
    last_updated_pub_ = create_publisher<TimeMsg>(topics::kLastUpdatedPub, 10);
}

VisionFilter::WorldStateMsg VisionFilter::BuildWorldStateMsg() const {
    const bool us_blue = config_client_.gameState().blue_team;

    WorldStateMsg msg{};
    msg.ball = BuildBallStateMsg();
    msg.our_robots = BuildRobotStateMsgs(us_blue);
    msg.their_robots = BuildRobotStateMsgs(!us_blue);

    return msg;
}

VisionFilter::BallStateMsg VisionFilter::BuildBallStateMsg() const {
    std::lock_guard<std::mutex> world_lock(world_mutex);
    const WorldBall& wb = world.getWorldBall();

    BallStateMsg msg{};
    msg.stamp = RJ::ToROSTime(wb.getTime());
    msg.position = wb.getPos();
    msg.velocity = wb.getVel();
    msg.visible = wb.getIsValid();
    return msg;
}

std::vector<VisionFilter::RobotStateMsg> VisionFilter::BuildRobotStateMsgs(
    bool blue_team) const {
    std::lock_guard<std::mutex> world_lock(world_mutex);
    const auto& robots =
        blue_team ? world.getRobotsBlue() : world.getRobotsYellow();

    // Fill our robots
    std::vector<RobotStateMsg> robot_state_msgs(Num_Shells);
    for (size_t i = 0; i < Num_Shells; i++) {
        const WorldRobot& wr = robots.at(i);

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();

        if (wr.getIsValid()) {
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity =
                Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        robot_state_msgs.at(i) = robot_state;
    }
    return robot_state_msgs;
}

void VisionFilter::PublishState() {
    if (!config_client_.connectedThreaded()) {
        return;
    }

    WorldStateMsg::UniquePtr msg = std::make_unique<WorldStateMsg>();
    *msg = BuildWorldStateMsg();
    world_state_pub_->publish(std::move(msg));

    if (last_update_time_) {
        last_updated_pub_->publish(RJ::ToROSTime(*last_update_time_));
    }
}

void VisionFilter::GetFrames() {
    std::vector<DetectionFrameMsg::UniquePtr> raw_frames =
        detection_frame_queue_.GetAll();
    if (raw_frames.empty()) {
        return;
    }

    last_update_time_ = RJ::FromROSTime(raw_frames.back()->t_received);

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
    const RJ::Seconds diff_duration =
        RJ::Seconds(PARAM_vision_loop_dt) - predict_time;

    if (diff_duration < RJ::Seconds{0}) {
        constexpr int kWarningThrottleMS = 1000;
        EZ_WARN_STREAM_THROTTLE(
            kWarningThrottleMS,
            "Predict is not called fast enough. Iteration took "
                << diff_duration.count() << " seconds, should be "
                << PARAM_vision_loop_dt << ".");
    }
}

void VisionFilter::PredictStatesImpl() {
    // Do update with whatever is in frame buffer
    GetFrames();
    std::lock_guard<std::mutex> world_lock(world_mutex);

    if (!frameBuffer.empty()) {
        world.updateWithCameraFrame(RJ::now(), frameBuffer);
        frameBuffer.clear();
    } else {
        world.updateWithoutCameraFrame(RJ::now());
    }
}
}  // namespace vision_filter
