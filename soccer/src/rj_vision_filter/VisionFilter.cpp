#include <Robot.hpp>
#include <iostream>
#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_vision_filter/VisionFilter.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {
DEFINE_FLOAT64(kVisionFilterParamModule, publish_hz, 120.0,
               "The rate in Hz at which VisionFilter publishes ball and robot "
               "observations.")
DEFINE_FLOAT64(kVisionFilterParamModule, predict_hz, 120.0,
               "The rate in Hz at which VisionFilter runs the prediction loop "
               "in each Kalman filter.")

VisionFilter::VisionFilter(const rclcpp::NodeOptions& options)
    : rclcpp::Node{"vision_filter", options},
      param_provider_{this, kVisionFilterParamModule} {
    threadEnd.store(false, std::memory_order::memory_order_seq_cst);

    rclcpp::executors::MultiThreadedExecutor executor;

    // Have to be careful so the entire initialization list
    // is created before the thread starts
    worker = std::thread(&VisionFilter::updateLoop, this);

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
    const std::chrono::duration<double> predict_timer_period(1 /
                                                             PARAM_predict_hz);
    predict_timer_ = create_wall_timer(
        publish_timer_period, [this]() { PublishState(); },
        predict_timer_callback_group_);
}

VisionFilter::~VisionFilter() {
    // Signal end of thread
    threadEnd.store(true, std::memory_order::memory_order_seq_cst);

    // Wait for it to die
    worker.join();
}

void VisionFilter::PublishState() {
    // Fill the list of our robots/balls based on whether we are the blue team
    // or not
    fillBallState(context_->state);
    fillRobotState(context_->state, context_->game_state.blueTeam);
}

void VisionFilter::GetFrames() {
    std::vector<DetectionFrameMsg::UniquePtr> raw_frames =
        detection_frame_sub_->GetFrames();
    if (raw_frames.empty()) {
        return;
    }

    last_update_time_ = RJ::FromROSTime(raw_frames.back()->t_received);

    const double team_angle = detection_frame_sub_->TeamAngle();
    const Geometry2d::TransformMatrix world_to_team =
        detection_frame_sub_->WorldToTeam();
    for (const DetectionFrameMsg::UniquePtr& msg : raw_frames) {
        frameBuffer.emplace_back(*msg, world_to_team, team_angle);
    }
}

void VisionFilter::fillBallState(SystemState& state) {
    std::lock_guard<std::mutex> lock(worldLock);
    const WorldBall& wb = world.getWorldBall();

    if (wb.getIsValid()) {
        state.ball.valid = true;
        state.ball.pos = wb.getPos();
        state.ball.vel = wb.getVel();
        state.ball.time = wb.getTime();
    } else {
        state.ball.valid = false;
    }
}

void VisionFilter::fillRobotState(SystemState& state, bool usBlue) {
    std::lock_guard<std::mutex> lock(worldLock);
    const auto& ourWorldRobot =
        usBlue ? world.getRobotsBlue() : world.getRobotsYellow();
    const auto& oppWorldRobot =
        usBlue ? world.getRobotsYellow() : world.getRobotsBlue();

    // Fill our robots
    for (int i = 0; i < Num_Shells; i++) {
        OurRobot* robot = state.self.at(i);
        const WorldRobot& wr = ourWorldRobot.at(i);

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();
        robot_state.velocity_valid = wr.getIsValid();

        if (wr.getIsValid()) {
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity =
                Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        robot->mutable_state() = robot_state;
    }

    // Fill opp robots
    for (int i = 0; i < Num_Shells; i++) {
        OpponentRobot* robot = state.opp.at(i);
        const WorldRobot& wr = oppWorldRobot.at(i);

        RobotState robot_state;
        robot_state.visible = wr.getIsValid();
        robot_state.velocity_valid = wr.getIsValid();

        if (wr.getIsValid()) {
            robot_state.pose = Geometry2d::Pose(wr.getPos(), wr.getTheta());
            robot_state.velocity =
                Geometry2d::Twist(wr.getVel(), wr.getOmega());
            robot_state.timestamp = wr.getTime();
        }

        robot->mutable_state() = robot_state;
    }
}

void VisionFilter::updateLoop() {
    while (!threadEnd.load(std::memory_order::memory_order_seq_cst)) {
        RJ::Time start = RJ::now();

        {
            // Do update with whatever is in frame buffer
            GetFrames();
            std::lock_guard<std::mutex> lock2(worldLock);

            if (!frameBuffer.empty()) {
                world.updateWithCameraFrame(RJ::now(), frameBuffer);
                frameBuffer.clear();
            } else {
                world.updateWithoutCameraFrame(RJ::now());
            }
        }

        // Wait for the correct loop timings
        RJ::Seconds diff = RJ::now() - start;
        RJ::Seconds sleepLeft =
            RJ::Seconds(*VisionFilterConfig::vision_loop_dt) - diff;

        if (diff > RJ::Seconds(0)) {
            std::this_thread::sleep_for(sleepLeft);
        } else {
            std::cout << "WARNING : Filter is not running fast enough"
                      << std::endl;
        }
    }
}
}  // namespace vision_filter
