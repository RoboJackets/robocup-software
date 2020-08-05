#include <cmath>
#include <deque>
#include <iterator>
#include <limits>

#include <Geometry2d/Point.hpp>
#include <rj_vision_filter/kick/detector/FastKickDetector.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, kick::detector,
                  fast_acceleration_trigger, 750.0,
                  "How large of an acceleration is needed to trigger this "
                  "detector in m/s^2. ")
using kick::detector::PARAM_fast_acceleration_trigger;

bool FastKickDetector::addRecord(RJ::Time calc_time, const WorldBall& ball,
                                 const std::vector<WorldRobot>& yellow_robots,
                                 const std::vector<WorldRobot>& blue_robots,
                                 KickEvent& kick_event) {
    // Keep it a certain length
    stateHistory.emplace_back(calc_time, ball, yellow_robots, blue_robots);
    if (stateHistory.size() >
        static_cast<size_t>(kick::detector::PARAM_fast_kick_hist_length)) {
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

    // If we didn't kick, just return
    if (!detectKick()) {
        return false;
    }

    // Assume the kick happened in the middle of the history
    int mid_idx = static_cast<int>(std::floor(stateHistory.size() / 2));

    WorldRobot closest_robot = getClosestRobot();
    RJ::Time kick_time = stateHistory.at(mid_idx).calcTime;
    std::deque<VisionState> states_since_kick(
        std::next(stateHistory.begin(), mid_idx), stateHistory.end());

    kick_event = KickEvent(kick_time, closest_robot, states_since_kick);

    return true;
}

bool FastKickDetector::detectKick() {
    // Note: This may be weird at camera frame intersections
    // It may be a good idea to look at the kalman balls
    // and checking kalman balls for velocity jumps

    // Returns true on a large velocity jump across the first and last
    // velocity calc
    int end_idx = stateHistory.size() - 1;

    // Change in position between two adjacent measurements
    Geometry2d::Point dp_start =
        stateHistory.at(1).ball.getPos() - stateHistory.at(0).ball.getPos();
    Geometry2d::Point dp_end = stateHistory.at(end_idx).ball.getPos() -
                               stateHistory.at(end_idx - 1).ball.getPos();

    // Velocity at the start and end measurements
    Geometry2d::Point v_start = dp_start / PARAM_vision_loop_dt;
    Geometry2d::Point v_end = dp_end / PARAM_vision_loop_dt;

    // Change in velocity between start and end measurements
    Geometry2d::Point dv = v_end - v_start;

    // Acceleration between the start and final velocity
    // This is weird when the history length is > 3, but it allows you not to
    // have to retune it
    Geometry2d::Point accel = dv / (PARAM_vision_loop_dt * stateHistory.size());

    // Check for large accelerations and only going from slow->fast transitions
    return accel.mag() > PARAM_fast_acceleration_trigger &&
           v_start.mag() < v_end.mag();
}

WorldRobot FastKickDetector::getClosestRobot() {
    // Get's the closest robot to the ball position in the center measurement
    // Assumes kick is in the center
    // Valid assumption as long as history length is small

    int mid_idx = (int)floor(stateHistory.size() / 2);
    Geometry2d::Point mid_ball_pos = stateHistory.at(mid_idx).ball.getPos();

    WorldRobot min_robot;
    double min_dist = std::numeric_limits<double>::infinity();

    // Finds closest robot to ball at assumed kick time
    for (WorldRobot& robot : stateHistory.at(mid_idx).yellowRobots) {
        if (robot.getIsValid()) {
            double dist = (mid_ball_pos - robot.getPos()).mag();

            if (dist < min_dist) {
                min_robot = robot;
            }
        }
    }

    for (WorldRobot& robot : stateHistory.at(mid_idx).blueRobots) {
        if (robot.getIsValid()) {
            double dist = (mid_ball_pos - robot.getPos()).mag();

            if (dist < min_dist) {
                min_robot = robot;
            }
        }
    }

    return min_robot;
}
}  // namespace vision_filter