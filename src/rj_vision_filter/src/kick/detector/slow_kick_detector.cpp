#include <algorithm>
#include <cmath>

#include <rj_geometry/point.hpp>
#include <rj_vision_filter/kick/detector/slow_kick_detector.hpp>

namespace vision_filter {

SlowKickDetector::SlowKickDetector(const std::shared_ptr<rclcpp::Node>& vision_filter_node) {
    vision_filter_node->get_parameter<double>("kick.detector.slow_robot_dist_filter_cutoff", slow_robot_dist_filter_cutoff_);
    vision_filter_node->get_parameter<double>("kick.detector.slow_one_robot_within_dist", slow_one_robot_within_dist_);
    vision_filter_node->get_parameter<double>("kick.detector.slow_any_robot_past_dist", slow_any_robot_past_dist_);
    vision_filter_node->get_parameter<double>("kick.detector.slow_min_ball_speed", slow_min_ball_speed_);
    vision_filter_node->get_parameter<double>("kick.detector.slow_max_kick_angle", slow_max_kick_angle_);
    vision_filter_node->get_parameter<int>("kick.detector.slow_kick_hist_length", slow_kick_hist_length_);
    vision_filter_node->get_parameter<int>("kick.detector.fast_kick_hist_length", fast_kick_hist_length_);
    vision_filter_node->get_parameter<double>("vision_loop_dt", vision_loop_dt_);
    
    param_cb_handle_ = vision_filter_node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            if (param.get_name() == "kick.detector.slow_robot_dist_filter_cutoff") {
                slow_robot_dist_filter_cutoff_ = param.as_double();
            } else if (param.get_name() == "kick.detector.slow_one_robot_with_dist") {
                slow_one_robot_within_dist_ = param.as_double();
            } else if (param.get_name() == "kick.detector.slow_any_robot_past_dist") {
                slow_any_robot_past_dist_ = param.as_double();
            } else if (param.get_name() == "kick.detector.slow_min_ball_speed") {
                slow_min_ball_speed_ = param.as_double();
            } else if (param.get_name() == "kick.detector.slow_max_kick_angle") {
                slow_max_kick_angle_ = param.as_double();
            } else if (param.get_name() == "kick.detector.slow_kick_hist_length") {
                slow_kick_hist_length_ = static_cast<int>(param.as_int());
            } else if (param.get_name() == "kick.detector.fast_kick_hist_length") {
                fast_kick_hist_length_ = static_cast<int>(param.as_int());
            } else if (param.get_name() == "vision_loop_dt") {
                vision_loop_dt_ = param.as_double();
            }
        }

        return result;
    });
}

bool SlowKickDetector::add_record(RJ::Time calc_time, const WorldBall& ball,
                                  const std::vector<WorldRobot>& yellow_robots,
                                  const std::vector<WorldRobot>& blue_robots,
                                  KickEvent* kick_event) {
    // Keep it a certain length
    state_history_.emplace_back(calc_time, ball, yellow_robots, blue_robots);
    if (state_history_.size() > static_cast<size_t>(slow_kick_hist_length_)) {
        state_history_.pop_front();
    }

    // If we don't have enough, just return
    if (state_history_.size() < static_cast<size_t>(fast_kick_hist_length_)) {
        return false;
    }

    // Make sure all the balls are valid
    // Otherwise we can't do anything
    bool all_valid = std::all_of(state_history_.begin(), state_history_.end(),
                                 [](VisionState& v) { return v.ball.get_is_valid(); }); //NOLINT(readability-identifier-length)

    if (!all_valid) {
        return false;
    }

    return detect_kick(kick_event);
}

bool SlowKickDetector::detect_kick(KickEvent* kick_event) const {
    // Find all the robots who have enough samples
    // Cut out any that are too far
    // Test validators on all of them

    std::vector<WorldBall> ball_list(state_history_.size());
    for (size_t i = 0; i < state_history_.size(); i++) {
        ball_list.at(i) = state_history_.at(i).ball;
    }

    // Check all robots if they have enough measurements
    for (size_t i = 0; i < state_history_.at(0).yellow_robots.size(); i++) {
        bool all_valid =
            std::all_of(state_history_.begin(), state_history_.end(),
                        [i](const VisionState& v) { return v.yellow_robots.at(i).get_is_valid(); }); //NOLINT(readability-identifier-length)

        // If not all the robots of this specific id are valid
        // check the next one
        if (!all_valid) {
            continue;
        }

        std::vector<WorldRobot> robot_list(state_history_.size());
        for (size_t j = 0; j < state_history_.size(); j++) {
            robot_list.at(j) = state_history_.at(j).yellow_robots.at(i);
        }

        // Valid kick robot
        // Just take this and return a kick event
        if (check_all_validators(robot_list, ball_list)) {
            *kick_event = KickEvent(state_history_.at(0).calc_time,
                                    state_history_.at(0).yellow_robots.at(i), state_history_);

            return true;
        }
    }

    // Check all robots if they have enough measurements
    // Same as above, need a better way to do this
    for (size_t i = 0; i < state_history_.at(0).blue_robots.size(); i++) {
        bool all_valid =
            std::all_of(state_history_.begin(), state_history_.end(),
                        [i](const VisionState& v) { return v.blue_robots.at(i).get_is_valid(); }); //NOLINT(readability-identifier-length)

        // If not all the robots of this specific id are valid
        // check the next one
        if (!all_valid) {
            continue;
        }

        std::vector<WorldRobot> robot_list(state_history_.size());
        for (size_t j = 0; j < state_history_.size(); j++) {
            robot_list.at(j) = state_history_.at(j).blue_robots.at(i);
        }

        // Valid kick robot
        // Just take this and return a kick event
        if (check_all_validators(robot_list, ball_list)) {
            *kick_event = KickEvent(state_history_.at(0).calc_time,
                                    state_history_.at(0).blue_robots.at(i), state_history_);

            return true;
        }
    }

    return false;
}

bool SlowKickDetector::check_all_validators(const std::vector<WorldRobot>& robot,
                                            const std::vector<WorldBall>& ball) const {
    return distance_validator(robot, ball) && velocity_validator(robot, ball) &&
           distance_increasing_validator(robot, ball) && in_front_validator(robot, ball);
}

bool SlowKickDetector::distance_validator(const std::vector<WorldRobot>& robot,
                                          const std::vector<WorldBall>& ball) const {
    // Make sure the first one is very close
    // And all the others are not
    // and if one or more are past the far distance

    // robot and ball are supposed to be the same size so this is ok
    std::vector<double> dist(robot.size(), 0);

    for (size_t i = 0; i < robot.size(); i++) {
        dist.at(i) = (robot.at(i).get_pos() - ball.at(i).get_pos()).mag();
    }

    //NOLINTNEXTLINE
    int num_close = std::count_if(dist.begin(), dist.end(),
                                  [this](double i) { return i < slow_one_robot_within_dist_; }); //NOLINT(readability-identifier-length)
    //NOLINTNEXTLINE
    int num_far = std::count_if(dist.begin(), dist.end(),
                                [this](double i) { return i > slow_any_robot_past_dist_; }); //NOLINT(readability-identifier-length)

    return num_close == 1 && num_far > 0;
}

bool SlowKickDetector::velocity_validator(const std::vector<WorldRobot>& /*robot*/,
                                          const std::vector<WorldBall>& ball) const {
    // Make sure all ball velocities are above a certain amount

    std::vector<double> vel(ball.size() - 1, 0);

    for (size_t i = 0; i < ball.size() - 1; i++) {
        vel.at(i) = (ball.at(i + 1).get_pos() - ball.at(i).get_pos()).mag() / vision_loop_dt_;
    }

    bool all_above =
        std::all_of(vel.begin(), vel.end(), [this](double i) { return i > slow_min_ball_speed_; }); //NOLINT(readability-identifier-length)

    return all_above;
}

bool SlowKickDetector::distance_increasing_validator(const std::vector<WorldRobot>& robot,
                                                     const std::vector<WorldBall>& ball) {
    // Make sure derivative of position is positive

    // robot and ball are the same size
    for (size_t i = 0; i < robot.size() - 1; i++) {
        double dist1 = (robot.at(i).get_pos() - ball.at(i).get_pos()).magsq();
        double dist2 = (robot.at(i + 1).get_pos() - ball.at(i + 1).get_pos()).magsq();

        if (dist2 - dist1 < 0) {
            return false;
        }
    }

    return true;
}

bool SlowKickDetector::in_front_validator(const std::vector<WorldRobot>& robot,
                                          const std::vector<WorldBall>& ball) const {
    // Make sure the ball is within a certain angle of the mouth

    // robot and ball are the same
    for (size_t i = 0; i < robot.size(); i++) {
        rj_geometry::Point normal =
            rj_geometry::Point(cos(robot.at(i).get_theta()), sin(robot.at(i).get_theta()));

        rj_geometry::Point robot_to_ball = ball.at(i).get_pos() - robot.at(i).get_pos();

        double angle = normal.angle_between(robot_to_ball);

        if (angle > slow_max_kick_angle_) {
            return false;
        }
    }

    return true;
}
}  // namespace vision_filter