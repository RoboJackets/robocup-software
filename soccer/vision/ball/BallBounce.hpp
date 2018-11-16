#pragma once

#include <vector>

#include "KalmanBall.hpp"
#include "vision/robot/WorldRobot.hpp"

class BallBounce {
    /**
     * Calculates whether the given kalman ball will bounce against another robot and
     * the resulting velocity vector
     *
     * @param ball Kalman ball we are trying to test
     * @param yellowRobots Best estimation of the yellow robots states
     * @param blueRobots Best esetimation of the yellow robots states
     * @param outNewVel Output of the resulting velocity vector after bounce
     *
     * @return Whether the ball bounces or not
     */
    static bool CalcBallBounce(KalmanBall& ball,
                               std::vector<WorldRobot>& yellowRobots,
                               std::vector<WorldRobot>& blueRobots,
                               Geometry2d::Point& outNewVel);
};