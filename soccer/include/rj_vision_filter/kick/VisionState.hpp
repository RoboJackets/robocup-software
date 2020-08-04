#pragma once

#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>
#include <utility>
#include <vector>

namespace vision_filter {
/**
 * Snapshot of the state of all objects in vision at a specific time
 */
class VisionState {
public:
    /**
     * @param calcTime Current frame time
     * @param ball Current frame ball
     * @param yellowRobots Current frame robot list
     * @param blueRobots Current frame robot list
     */
    VisionState(RJ::Time calcTime, WorldBall ball,
                std::vector<WorldRobot> yellowRobots,
                std::vector<WorldRobot> blueRobots)
        : calcTime(calcTime),
          ball(std::move(ball)),
          yellowRobots(std::move(yellowRobots)),
          blueRobots(std::move(blueRobots)) {}

    RJ::Time calcTime;
    WorldBall ball;
    std::vector<WorldRobot> yellowRobots;
    std::vector<WorldRobot> blueRobots;
};
}  // namespace vision_filter
