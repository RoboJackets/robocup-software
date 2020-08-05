#include <algorithm>
#include <cmath>

#include <Geometry2d/Point.hpp>
#include <rj_vision_filter/kick/detector/SlowKickDetector.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, kick::detector, slow_robot_dist_filter_cutoff,
    3.0, "Doesn't check any robots past this distance in m for optimization.")
DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, kick::detector, slow_one_robot_within_dist, 0.15,
    "Only one ball measurement within this distance of the robot.")
DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, kick::detector, slow_any_robot_past_dist, 0.16,
    "At least one ball measurement past this distance of the robot.")
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector, slow_min_ball_speed,
                  0.6, "Ball has to be this fast.")
DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, kick::detector, slow_max_kick_angle, 0.34,
    "Max angle difference between velocity vector and robot heading.")
using namespace kick::detector;

bool SlowKickDetector::addRecord(RJ::Time calc_time, const WorldBall& ball,
                                 const std::vector<WorldRobot>& yellow_robots,
                                 const std::vector<WorldRobot>& blue_robots,
                                 KickEvent* kick_event) {
    // Keep it a certain length
    stateHistory.emplace_back(calc_time, ball, yellow_robots, blue_robots);
    if (stateHistory.size() >
        static_cast<size_t>(kick::detector::PARAM_slow_kick_hist_length)) {
        stateHistory.pop_front();
    }

    // If we don't have enough, just return
    if (stateHistory.size() <
        static_cast<size_t>(kick::detector::PARAM_fast_kick_hist_length)) {
        return false;
    }

    // Make sure all the balls are valid
    // Otherwise we can't do anything
    bool all_valid =
        std::all_of(stateHistory.begin(), stateHistory.end(),
                    [](VisionState& v) { return v.ball.getIsValid(); });

    if (!all_valid) {
        return false;
    }

    return detectKick(kick_event);
}

bool SlowKickDetector::detectKick(KickEvent* kick_event) {
    // Find all the robots who have enough samples
    // Cut out any that are too far
    // Test validators on all of them

    std::vector<WorldBall> ball_list(stateHistory.size());
    for (int i = 0; i < stateHistory.size(); i++) {
        ball_list.at(i) = stateHistory.at(i).ball;
    }

    // Check all robots if they have enough measurements
    for (int i = 0; i < stateHistory.at(0).yellowRobots.size(); i++) {
        bool all_valid = std::all_of(
            stateHistory.begin(), stateHistory.end(),
            [i](VisionState& v) { return v.yellowRobots.at(i).getIsValid(); });

        // If not all the robots of this specific id are valid
        // check the next one
        if (!all_valid) {
            continue;
        }

        std::vector<WorldRobot> robot_list(stateHistory.size());
        for (int j = 0; j < stateHistory.size(); j++) {
            robot_list.at(j) = stateHistory.at(j).yellowRobots.at(i);
        }

        // Valid kick robot
        // Just take this and return a kick event
        if (checkAllValidators(robot_list, ball_list)) {
            *kick_event =
                KickEvent(stateHistory.at(0).calcTime,
                          stateHistory.at(0).yellowRobots.at(i), stateHistory);

            return true;
        }
    }

    // Check all robots if they have enough measurements
    // Same as above, need a better way to do this
    for (int i = 0; i < stateHistory.at(0).blueRobots.size(); i++) {
        bool all_valid = std::all_of(
            stateHistory.begin(), stateHistory.end(),
            [i](VisionState& v) { return v.blueRobots.at(i).getIsValid(); });

        // If not all the robots of this specific id are valid
        // check the next one
        if (!all_valid) {
            continue;
        }

        std::vector<WorldRobot> robot_list(stateHistory.size());
        for (int j = 0; j < stateHistory.size(); j++) {
            robot_list.at(j) = stateHistory.at(j).blueRobots.at(i);
        }

        // Valid kick robot
        // Just take this and return a kick event
        if (checkAllValidators(robot_list, ball_list)) {
            *kick_event =
                KickEvent(stateHistory.at(0).calcTime,
                          stateHistory.at(0).blueRobots.at(i), stateHistory);

            return true;
        }
    }

    return false;
}

bool SlowKickDetector::checkAllValidators(const std::vector<WorldRobot>& robot,
                                          const std::vector<WorldBall>& ball) {
    return distanceValidator(robot, ball) && velocityValidator(robot, ball) &&
           distanceIncreasingValidator(robot, ball) &&
           inFrontValidator(robot, ball);
}

bool SlowKickDetector::distanceValidator(const std::vector<WorldRobot>& robot,
                                         const std::vector<WorldBall>& ball) {
    // Make sure the first one is very close
    // And all the others are not
    // and if one or more are past the far distance

    // robot and ball are supposed to be the same size so this is ok
    std::vector<double> dist(robot.size(), 0);

    for (int i = 0; i < robot.size(); i++) {
        dist.at(i) = (robot.at(i).getPos() - ball.at(i).getPos()).mag();
    }

    int num_close = std::count_if(dist.begin(), dist.end(), [](double i) {
        return i < PARAM_slow_one_robot_within_dist;
    });
    int num_far = std::count_if(dist.begin(), dist.end(), [](double i) {
        return i > PARAM_slow_any_robot_past_dist;
    });

    return num_close == 1 && num_far > 0;
}

bool SlowKickDetector::velocityValidator(
    const std::vector<WorldRobot>& /*robot*/,
    const std::vector<WorldBall>& ball) {
    // Make sure all ball velocities are above a certain amount

    std::vector<double> vel(ball.size() - 1, 0);

    for (int i = 0; i < ball.size() - 1; i++) {
        vel.at(i) = (ball.at(i + 1).getPos() - ball.at(i).getPos()).mag() /
                    PARAM_vision_loop_dt;
    }

    bool all_above = std::all_of(vel.begin(), vel.end(), [](double i) {
        return i > PARAM_slow_min_ball_speed;
    });

    return all_above;
}

bool SlowKickDetector::distanceIncreasingValidator(
    const std::vector<WorldRobot>& robot, const std::vector<WorldBall>& ball) {
    // Make sure derivative of position is positive

    // robot and ball are the same size
    for (int i = 0; i < robot.size() - 1; i++) {
        double dist1 = (robot.at(i).getPos() - ball.at(i).getPos()).magsq();
        double dist2 =
            (robot.at(i + 1).getPos() - ball.at(i + 1).getPos()).magsq();

        if (dist2 - dist1 < 0) {
            return false;
        }
    }

    return true;
}

bool SlowKickDetector::inFrontValidator(const std::vector<WorldRobot>& robot,
                                        const std::vector<WorldBall>& ball) {
    // Make sure the ball is within a certain angle of the mouth

    // robot and ball are the same
    for (int i = 0; i < robot.size(); i++) {
        Geometry2d::Point normal = Geometry2d::Point(
            cos(robot.at(i).getTheta()), sin(robot.at(i).getTheta()));

        Geometry2d::Point robot_to_ball =
            ball.at(i).getPos() - robot.at(i).getPos();

        double angle = normal.angleBetween(robot_to_ball);

        if (angle > PARAM_slow_max_kick_angle) {
            return false;
        }
    }

    return true;
}
}  // namespace vision_filter