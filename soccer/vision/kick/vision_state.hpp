#pragma once

#include <utility>
#include <vector>

#include "vision/ball/world_ball.hpp"
#include "vision/robot/world_robot.hpp"

namespace vision_filter {
/**
 * Snapshot of the state of all objects in vision at a specific time
 */
class VisionState {
public:
    /**
     * @param calc_time Current frame time
     * @param ball Current frame ball
     * @param yellow_robots Current frame robot list
     * @param blue_robots Current frame robot list
     */
    VisionState(RJ::Time calc_time, WorldBall ball, std::vector<WorldRobot> yellow_robots,
                std::vector<WorldRobot> blue_robots)
        : calc_time(calc_time),
          ball(std::move(ball)),
          yellow_robots(std::move(yellow_robots)),
          blue_robots(std::move(blue_robots)) {}

    RJ::Time calc_time;
    WorldBall ball;
    std::vector<WorldRobot> yellow_robots;
    std::vector<WorldRobot> blue_robots;
};
}  // namespace vision_filter
