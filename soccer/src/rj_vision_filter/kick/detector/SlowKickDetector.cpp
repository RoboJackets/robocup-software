#include <Geometry2d/Point.hpp>
#include <algorithm>
#include <cmath>
#include <rj_vision_filter/kick/detector/SlowKickDetector.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

// TODO(?): The parameters below aren't used anywhere.
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

bool SlowKickDetector::addRecord(RJ::Time calcTime, const WorldBall& ball,
                                 const std::vector<WorldRobot>& yellowRobots,
                                 const std::vector<WorldRobot>& blueRobots,
                                 KickEvent& kickEvent) {
    // Keep it a certain length
    stateHistory.emplace_back(calcTime, ball, yellowRobots, blueRobots);
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
    bool allValid =
        std::all_of(stateHistory.begin(), stateHistory.end(),
                    [](VisionState& v) { return v.ball.getIsValid(); });

    if (!allValid) {
        return false;
    }

    return detectKick(kickEvent);
}

bool SlowKickDetector::detectKick(KickEvent& kickEvent) {
    // Find all the robots who have enough samples
    // Cut out any that are too far
    // Test validators on all of them

    std::vector<WorldBall> ballList(stateHistory.size());
    for (int i = 0; i < stateHistory.size(); i++) {
        ballList.at(i) = stateHistory.at(i).ball;
    }

    // Check all robots if they have enough measurements
    for (int i = 0; i < stateHistory.at(0).yellowRobots.size(); i++) {
        bool allValid = std::all_of(
            stateHistory.begin(), stateHistory.end(),
            [i](VisionState& v) { return v.yellowRobots.at(i).getIsValid(); });

        // If not all the robots of this specific id are valid
        // check the next one
        if (!allValid) {
            continue;
        }

        std::vector<WorldRobot> robotList(stateHistory.size());
        for (int j = 0; j < stateHistory.size(); j++) {
            robotList.at(j) = stateHistory.at(j).yellowRobots.at(i);
        }

        // Valid kick robot
        // Just take this and return a kick event
        if (checkAllValidators(robotList, ballList)) {
            kickEvent =
                KickEvent(stateHistory.at(0).calcTime,
                          stateHistory.at(0).yellowRobots.at(i), stateHistory);

            return true;
        }
    }

    // Check all robots if they have enough measurements
    // Same as above, need a better way to do this
    for (int i = 0; i < stateHistory.at(0).blueRobots.size(); i++) {
        bool allValid = std::all_of(
            stateHistory.begin(), stateHistory.end(),
            [i](VisionState& v) { return v.blueRobots.at(i).getIsValid(); });

        // If not all the robots of this specific id are valid
        // check the next one
        if (!allValid) {
            continue;
        }

        std::vector<WorldRobot> robotList(stateHistory.size());
        for (int j = 0; j < stateHistory.size(); j++) {
            robotList.at(j) = stateHistory.at(j).blueRobots.at(i);
        }

        // Valid kick robot
        // Just take this and return a kick event
        if (checkAllValidators(robotList, ballList)) {
            kickEvent =
                KickEvent(stateHistory.at(0).calcTime,
                          stateHistory.at(0).blueRobots.at(i), stateHistory);

            return true;
        }
    }

    return false;
}

bool SlowKickDetector::checkAllValidators(std::vector<WorldRobot>& robot,
                                          std::vector<WorldBall>& ball) {
    return distanceValidator(robot, ball) && velocityValidator(robot, ball) &&
           distanceIncreasingValidator(robot, ball) &&
           inFrontValidator(robot, ball);
}

bool SlowKickDetector::distanceValidator(std::vector<WorldRobot>& robot,
                                         std::vector<WorldBall>& ball) {
    // Make sure the first one is very close
    // And all the others are not
    // and if one or more are past the far distance

    // robot and ball are supposed to be the same size so this is ok
    std::vector<double> dist(robot.size(), 0);

    for (int i = 0; i < robot.size(); i++) {
        dist.at(i) = (robot.at(i).getPos() - ball.at(i).getPos()).mag();
    }

    int numClose = std::count_if(dist.begin(), dist.end(), [](double i) {
        return i < *one_robot_within_dist;
    });
    int numFar = std::count_if(dist.begin(), dist.end(), [](double i) {
        return i > *any_robot_past_dist;
    });

    return numClose == 1 && numFar > 0;
}

bool SlowKickDetector::velocityValidator(std::vector<WorldRobot>& /*robot*/,
                                         std::vector<WorldBall>& ball) {
    // Make sure all ball velocities are above a certain amount

    std::vector<double> vel(ball.size() - 1, 0);

    for (int i = 0; i < ball.size() - 1; i++) {
        vel.at(i) = (ball.at(i + 1).getPos() - ball.at(i).getPos()).mag() /
                    PARAM_vision_loop_dt;
    }

    bool allAbove = std::all_of(vel.begin(), vel.end(),
                                [](double i) { return i > *min_ball_speed; });

    return allAbove;
}

bool SlowKickDetector::distanceIncreasingValidator(
    std::vector<WorldRobot>& robot, std::vector<WorldBall>& ball) {
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

bool SlowKickDetector::inFrontValidator(std::vector<WorldRobot>& robot,
                                        std::vector<WorldBall>& ball) {
    // Make sure the ball is within a certain angle of the mouth

    // robot and ball are the same
    for (int i = 0; i < robot.size(); i++) {
        Geometry2d::Point normal = Geometry2d::Point(
            cos(robot.at(i).getTheta()), sin(robot.at(i).getTheta()));

        Geometry2d::Point robotToBall =
            ball.at(i).getPos() - robot.at(i).getPos();

        double angle = normal.angleBetween(robotToBall);

        if (angle > *max_kick_angle) {
            return false;
        }
    }

    return true;
}
}  // namespace vision_filter