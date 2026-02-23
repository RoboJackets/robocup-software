#include <algorithm>
#include <cmath>

#include <rj_geometry/point.hpp>
#include <rj_vision_filter/kick/detector/slow_kick_detector.hpp>

namespace vision_filter {

bool SlowKickDetector::add_record(RJ::Time calc_time, const WorldBall& ball,
                                  const std::vector<WorldRobot>& yellow_robots,
                                  const std::vector<WorldRobot>& blue_robots,
                                  KickEvent* kick_event) {
    // Keep it a certain length
    state_history_.emplace_back(calc_time, ball, yellow_robots, blue_robots);
    if (state_history_.size() > static_cast<size_t>(config_->kick_detector.slow_kick_hist_length)) {
        state_history_.pop_front();
    }

    // If we don't have enough, just return
    if (state_history_.size() < static_cast<size_t>(config_->kick_detector.fast_kick_hist_length)) {
        return false;
    }

    // Make sure all the balls are valid
    // Otherwise we can't do anything
    bool all_valid = std::all_of(state_history_.begin(), state_history_.end(),
                                 [](VisionState& v) { return v.ball.get_is_valid(); });

    if (!all_valid) {
        return false;
    }

    return detect_kick(kick_event);
}

bool SlowKickDetector::detect_kick(KickEvent* kick_event) {
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
                        [i](VisionState& v) { return v.yellow_robots.at(i).get_is_valid(); });

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
        if (check_all_validators(robot_list, ball_list, *config_)) {
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
                        [i](VisionState& v) { return v.blue_robots.at(i).get_is_valid(); });

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
        if (check_all_validators(robot_list, ball_list, *config_)) {
            *kick_event = KickEvent(state_history_.at(0).calc_time,
                                    state_history_.at(0).blue_robots.at(i), state_history_);

            return true;
        }
    }

    return false;
}

bool SlowKickDetector::check_all_validators(const std::vector<WorldRobot>& robot,
                                            const std::vector<WorldBall>& ball,
                                            const VisionFilterConfig& config) {
    return distance_validator(robot, ball, config) && velocity_validator(robot, ball, config) &&
           distance_increasing_validator(robot, ball) && in_front_validator(robot, ball, config);
}

bool SlowKickDetector::distance_validator(const std::vector<WorldRobot>& robot,
                                          const std::vector<WorldBall>& ball,
                                          const VisionFilterConfig& config) {
    // Make sure the first one is very close
    // And all the others are not
    // and if one or more are past the far distance

    // robot and ball are supposed to be the same size so this is ok
    std::vector<double> dist(robot.size(), 0);

    for (size_t i = 0; i < robot.size(); i++) {
        dist.at(i) = (robot.at(i).get_pos() - ball.at(i).get_pos()).mag();
    }

    int num_close = std::count_if(dist.begin(), dist.end(),
                                  [&config](double i) { return i < config.kick_detector.slow_one_robot_within_dist; });
    int num_far = std::count_if(dist.begin(), dist.end(),
                                [&config](double i) { return i > config.kick_detector.slow_any_robot_past_dist; });

    return num_close == 1 && num_far > 0;
}

bool SlowKickDetector::velocity_validator(const std::vector<WorldRobot>& /*robot*/,
                                          const std::vector<WorldBall>& ball,
                                          const VisionFilterConfig& config) {
    // Make sure all ball velocities are above a certain amount

    std::vector<double> vel(ball.size() - 1, 0);

    for (size_t i = 0; i < ball.size() - 1; i++) {
        vel.at(i) = (ball.at(i + 1).get_pos() - ball.at(i).get_pos()).mag() / config.vision_loop_dt;
    }

    bool all_above =
        std::all_of(vel.begin(), vel.end(), [&config](double i) { return i > config.kick_detector.slow_min_ball_speed; });

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
                                          const std::vector<WorldBall>& ball,
                                          const VisionFilterConfig& config) {
    // Make sure the ball is within a certain angle of the mouth

    // robot and ball are the same
    for (size_t i = 0; i < robot.size(); i++) {
        rj_geometry::Point normal =
            rj_geometry::Point(cos(robot.at(i).get_theta()), sin(robot.at(i).get_theta()));

        rj_geometry::Point robot_to_ball = ball.at(i).get_pos() - robot.at(i).get_pos();

        double angle = normal.angle_between(robot_to_ball);

        if (angle > config.kick_detector.slow_max_kick_angle) {
            return false;
        }
    }

    return true;
}
}  // namespace vision_filter