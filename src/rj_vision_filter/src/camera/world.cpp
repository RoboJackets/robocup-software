#include <rj_constants/constants.hpp>
#include <rj_vision_filter/camera/world.hpp>

namespace vision_filter {

World::World(std::shared_ptr<rclcpp::Node> vision_filter_node)
    : last_update_time_{RJ::Time{RJ::Time::duration(0)}},
      robots_yellow_(kNumShells, WorldRobot()),
      robots_blue_(kNumShells, WorldRobot()),
      fast_kick_(vision_filter_node),
      slow_kick_(vision_filter_node),
      vision_filter_node_(std::move(vision_filter_node)) {
    initialize_parameters();

    param_cb_handle_ = vision_filter_node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = update_parameters(params);

        return result;
    });
}

void World::update_single_camera(RJ::Time calc_time, const CameraFrame& frame) {
    update_with_camera_frame(calc_time, {
        frame}, false);
}

void World::update_with_camera_frame(RJ::Time calc_time, const std::vector<CameraFrame>& new_frames,
                                     bool update_all) {
    calc_ball_bounce();

    std::vector<bool> camera_updated(cameras_.size(), false);

    // TODO (UNKNOWN): Take only the newest frame if 2 come in for the same camera

    for (const CameraFrame& frame : new_frames) {
        // Make sure camera from frame is created, if not, make it
        if (!cameras_.at(frame.camera_id).get_is_valid()) {
            cameras_.at(frame.camera_id) = Camera(frame.camera_id, vision_filter_node_);
        }

        // Take the non-sorted list from the frame and make a list for the
        // cameras
        std::vector<std::list<CameraRobot>> yellow_team(kNumShells);
        std::vector<std::list<CameraRobot>> blue_team(kNumShells);

        for (const CameraRobot& robot : frame.camera_robots_yellow) {
            yellow_team.at(robot.get_robot_id()).push_back(robot);
        }

        for (const CameraRobot& robot : frame.camera_robots_blue) {
            blue_team.at(robot.get_robot_id()).push_back(robot);
        }

        cameras_.at(frame.camera_id)
            .update_with_frame(calc_time, frame.camera_balls, yellow_team, blue_team, ball_,
                               robots_yellow_, robots_blue_);

        camera_updated.at(frame.camera_id) = true;

        // Update last_update_time_ with the latest t_capture.
        last_update_time_ = std::max(last_update_time_, frame.t_capture);
    }

    if (update_all) {
        for (size_t i = 0; i < cameras_.size(); i++) {
            if (!camera_updated.at(i) && cameras_.at(i).get_is_valid()) {
                cameras_.at(i).update_without_frame(calc_time);
            }
        }
    }

    update_world_objects(calc_time);
    detect_kicks(calc_time);
}

void World::update_without_camera_frame(RJ::Time calc_time) {
    calc_ball_bounce();

    for (Camera& camera : cameras_) {
        if (camera.get_is_valid()) {
            camera.update_without_frame(calc_time);
        }
    }

    update_world_objects(calc_time);
    detect_kicks(calc_time);
}

void World::calc_ball_bounce() {
    for (Camera& camera : cameras_) {
        if (camera.get_is_valid()) {
            camera.process_ball_bounce(robots_yellow_, robots_blue_);
        }
    }
}

//NOLINTNEXTLINE(readability-function-cognitive-complexity)
void World::update_world_objects(RJ::Time calc_time) {
    // Fill robots_yellow_/blue with what robots we want and remove the rest
    ball_ = WorldBall();

    std::fill(robots_yellow_.begin(), robots_yellow_.end(), WorldRobot());
    std::fill(robots_blue_.begin(), robots_blue_.end(), WorldRobot());

    std::list<KalmanBall> kalman_balls;
    std::vector<std::list<KalmanRobot>> kalman_robots_yellow(kNumShells);
    std::vector<std::list<KalmanRobot>> kalman_robots_blue(kNumShells);

    // Take best kalman filter from every camera and combine them
    for (Camera& camera : cameras_) {
        if (camera.get_is_valid()) {
            std::list<KalmanBall> camera_balls = camera.get_kalman_balls();
            std::vector<std::list<KalmanRobot>> camera_robots_yellow =
                camera.get_kalman_robots_yellow();
            std::vector<std::list<KalmanRobot>> camera_robots_blue =
                camera.get_kalman_robots_blue();

            if (!camera_balls.empty()) {
                // Sort by health of the kalman filter
                camera_balls.sort([](KalmanBall& ball_a, KalmanBall& ball_b) -> bool {
                    return ball_a.get_health() > ball_b.get_health();
                });

                kalman_balls.push_back(camera_balls.front());
            }

            // Take the best kalman filter from the camera
            for (size_t i = 0; i < camera_robots_yellow.size(); i++) {
                if (!camera_robots_yellow.at(i).empty()) {
                    camera_robots_yellow.at(i).sort([](KalmanRobot& robot_a, KalmanRobot& robot_b) -> bool {
                        return robot_a.get_health() > robot_b.get_health();
                    });

                    kalman_robots_yellow.at(i).push_back(camera_robots_yellow.at(i).front());
                }
            }

            // Take the best kalman filter from the camera
            for (size_t i = 0; i < camera_robots_blue.size(); i++) {
                if (!camera_robots_blue.at(i).empty()) {
                    camera_robots_blue.at(i).sort([](KalmanRobot& robot_a, KalmanRobot& robot_b) -> bool {
                        return robot_a.get_health() > robot_b.get_health();
                    });

                    kalman_robots_blue.at(i).push_back(camera_robots_blue.at(i).front());
                }
            }
        }
    }

    // Only replace the invalid result if we have measurements on any camera
    if (!kalman_balls.empty()) {
        ball_ = WorldBall(calc_time, kalman_balls, ball_merger_power_);
    }

    for (size_t i = 0; i < robots_yellow_.size(); i++) {
        if (!kalman_robots_yellow.at(i).empty()) {
            robots_yellow_.at(i) =
                WorldRobot(calc_time, WorldRobot::Team::YELLOW, static_cast<int>(i), kalman_robots_yellow.at(i), robot_merger_power_);
        }
    }

    for (size_t i = 0; i < robots_blue_.size(); i++) {
        if (!kalman_robots_blue.at(i).empty()) {
            robots_blue_.at(i) =
                WorldRobot(calc_time, WorldRobot::Team::BLUE, static_cast<int>(i), kalman_robots_blue.at(i), robot_merger_power_);
        }
    }
}

void World::detect_kicks(RJ::Time calc_time) {
    KickEvent fast_event;
    KickEvent slow_event;

    bool is_fast_kick =
        fast_kick_.add_record(calc_time, ball_, robots_yellow_, robots_blue_, fast_event);
    bool is_slow_kick =
        slow_kick_.add_record(calc_time, ball_, robots_yellow_, robots_blue_, &slow_event);

    // If there isn't a kick recorded already
    if (!best_kick_estimate_.get_is_valid()) {
        // Try to use the slow kick as it's a better estimate
        // but take fast kick if there isn't a corrsponding slow kick yet
        if (is_slow_kick) {
            best_kick_estimate_ = slow_event;
        } else if (is_fast_kick) {
            best_kick_estimate_ = fast_event;
        }

        // There is a kick recorded already
    } else {
        const RJ::Seconds time_since_best_event(best_kick_estimate_.get_kick_time() - calc_time);

        // Try using the slow kick if:
        //      - It refers to the current best kick event (and probably is a
        //      better estimate)
        //      - The old kick timed out and should be updated
        if (is_slow_kick && (time_since_best_event < same_kick_timeout_ ||
                             time_since_best_event > slow_kick_timeout_)) {
            best_kick_estimate_ = slow_event;

            // Try using the fast kick if the old kick timed out
        } else if (is_fast_kick && time_since_best_event > fast_kick_timeout_) {
            best_kick_estimate_ = fast_event;

            // Remove the old kick if it's completely time out
        } else if (time_since_best_event > slow_kick_timeout_ + fast_kick_timeout_) {
            best_kick_estimate_ = KickEvent();
        }
    }
}

const WorldBall& World::get_world_ball() const { return ball_; }

const std::vector<WorldRobot>& World::get_robots_yellow() const { return robots_yellow_; }

const std::vector<WorldRobot>& World::get_robots_blue() const { return robots_blue_; }

const KickEvent& World::get_best_kick_estimate() const { return best_kick_estimate_; }

void World::initialize_parameters() {
    cameras_ = std::vector<Camera>(vision_filter_node_->get_parameter("max_num_cameras").as_int());
    same_kick_timeout_ = RJ::Seconds(vision_filter_node_->get_parameter("kick.detector.same_kick_timeout").as_double());
    slow_kick_timeout_ = RJ::Seconds(vision_filter_node_->get_parameter("kick.detector.slow_kick_timeout").as_double());
    fast_kick_timeout_ = RJ::Seconds(vision_filter_node_->get_parameter("kick.detector.fast_kick_timeout").as_double());
    vision_filter_node_->get_parameter<double>("world_ball.ball_merger_power", ball_merger_power_);
    vision_filter_node_->get_parameter<double>("world_robot.robot_merger_power", robot_merger_power_);
}

bool World::update_parameters(const std::vector<rclcpp::Parameter>& params) {
    bool success = true;

    for (const auto& param : params) {
        if (param.get_name() == "max_num_cameras") {
            success = false;
        } else if (param.get_name() == "kick.detector.same_kick_timeout") {
            same_kick_timeout_ = RJ::Seconds(param.get_value<double>());
        } else if (param.get_name() == "kick.detector.slow_kick_timeout") {
            slow_kick_timeout_ = RJ::Seconds(param.get_value<double>());
        } else if (param.get_name() == "kick.detector.fast_kick_timeout") {
            fast_kick_timeout_ = RJ::Seconds(param.get_value<double>());
        } else if (param.get_name() == "world_ball.ball_merger_power") {
            ball_merger_power_ = param.as_double();
        } else if (param.get_name() == "world_robot.robot_merger_power") {
            robot_merger_power_ = param.as_double();
        }
    }

    return success;
}

}  // namespace vision_filter
