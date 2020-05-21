#include "FastKickDetector.hpp"

#include <math.h>
#include <deque>
#include <iterator>
#include <limits>

#include <Geometry2d/Point.hpp>

#include "vision/util/VisionFilterConfig.hpp"

REGISTER_CONFIGURABLE(FastKickDetector)

ConfigDouble* FastKickDetector::acceleration_trigger;

void FastKickDetector::createConfiguration(Configuration* cfg) {
    acceleration_trigger = new ConfigDouble(cfg, "VisionFilter/Kick/Detector/fast_acceleration_trigger", 750);
}

bool FastKickDetector::addRecord(RJ::Time calcTime, WorldBall ball,
                                 std::vector<WorldRobot> yellowRobots,
                                 std::vector<WorldRobot> blueRobots,
                                 KickEvent& kickEvent) {
    // Keep it a certain length
    stateHistory.emplace_back(calcTime, ball, yellowRobots, blueRobots);
    if (stateHistory.size() > *VisionFilterConfig::fast_kick_detector_history_length) {
        stateHistory.pop_front();
    }

    // If we don't have enough, just return
    if (stateHistory.size() < *VisionFilterConfig::fast_kick_detector_history_length) {
        return false;
    }

    // Make sure all the balls are valid
    // Otherwise we can't do anything
    bool allValid = std::all_of(stateHistory.begin(), stateHistory.end(),
                                [](VisionState& v){
                                    return v.ball.getIsValid();
                                });

    if (!allValid) {
        return false;
    }

    // If we didn't kick, just return
    if (!detectKick()) {
        return false;
    }

    // Assume the kick happened in the middle of the history
    int midIdx = (int)floor(stateHistory.size() / 2);

    WorldRobot closestRobot = getClosestRobot();
    RJ::Time kickTime = stateHistory.at(midIdx).calcTime;
    std::deque<VisionState> statesSinceKick(std::next(stateHistory.begin(), midIdx), stateHistory.end());

    kickEvent = KickEvent(kickTime, closestRobot, statesSinceKick);

    return true;
}

bool FastKickDetector::detectKick() {
    // Note: This may be weird at camera frame intersections
    // It may be a good idea to look at the kalman balls
    // and checking kalman balls for velocity jumps

    // Returns true on a large velocity jump across the first and last
    // velocity calc
    int endIdx = stateHistory.size() - 1;

    // Change in position between two adjacent measurements
    Geometry2d::Point dpStart = stateHistory.at(1).ball.getPos() - stateHistory.at(0).ball.getPos();
    Geometry2d::Point dpEnd = stateHistory.at(endIdx).ball.getPos() - stateHistory.at(endIdx - 1).ball.getPos();

    // Velocity at the start and end measurements
    Geometry2d::Point vStart = dpStart / *VisionFilterConfig::vision_loop_dt;
    Geometry2d::Point vEnd = dpEnd / *VisionFilterConfig::vision_loop_dt;

    // Change in velocity between start and end measurements
    Geometry2d::Point dv = vEnd - vStart;

    // Acceleration between the start and final velocity
    // This is weird when the history length is > 3, but it allows you not to have to retune it
    Geometry2d::Point accel = dv / (*VisionFilterConfig::vision_loop_dt * stateHistory.size());

    // Check for large accelerations and only going from slow->fast transitions
    return accel.mag() > *acceleration_trigger && vStart.mag() < vEnd.mag();
}

WorldRobot FastKickDetector::getClosestRobot() {
    // Get's the closest robot to the ball position in the center measurement
    // Assumes kick is in the center
    // Valid assumption as long as history length is small

    int midIdx = (int)floor(stateHistory.size() / 2);
    Geometry2d::Point midBallPos = stateHistory.at(midIdx).ball.getPos();

    WorldRobot minRobot;
    double minDist = std::numeric_limits<double>::infinity();

    // Finds closest robot to ball at assumed kick time
    for (WorldRobot& robot : stateHistory.at(midIdx).yellowRobots) {
        if (robot.getIsValid()) {
            double dist = (midBallPos - robot.getPos()).mag();

            if (dist < minDist) {
                minRobot = robot;
            }
        }
    }

    for (WorldRobot& robot : stateHistory.at(midIdx).blueRobots) {
        if (robot.getIsValid()) {
            double dist = (midBallPos - robot.getPos()).mag();

            if (dist < minDist) {
                minRobot = robot;
            }
        }
    }

    return minRobot;
}