#pragma once

#include <deque>

#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "vision/kick/KickEvent.hpp"
#include "vision/kick/VisionState.hpp"

/**
 * Accruately detects kicks by robots using 5 or more samples
 * in history.
 *
 * Used in the general case when we have time to take a second
 * and double check results before sending it back
 */
class SlowKickDetector {
public:
    /**
     * Adds a record to our history list
     *
     * @param calcTime Time of calculation for this vision loop
     * @param ball Best estimation of the current ball
     * @param yellowRobots Best estimation of the yellow robots
     * @param blueRobots Best estimation of the blue robots
     * @param kickEvent Returned kick event if we find one
     *
     * @return Whether there was a kick
     *
     * @note kickEvent is only filled if it returns true
     * It will change, but will have invalid data in it
     */
    bool addRecord(RJ::Time calcTime, WorldBall ball,
                   std::vector<WorldRobot> yellowRobots,
                   std::vector<WorldRobot> blueRobots,
                   KickEvent& kickEvent);

    static void createConfiguration(Configuration* cfg);

private:
    /**
     * Tries to find out if/which robot kicked
     */
    bool detectKick(KickEvent& kickEvent);

    /**
     * Checks to see if all the different tests to detect kicks are true
     */
    bool checkAllValidators(std::vector<WorldRobot>& robot, std::vector<WorldBall>& ball);

    /**
     * If ball and robots were close and are now far away
     */
    bool distanceValidator(std::vector<WorldRobot>& robot, std::vector<WorldBall>& ball);

    /**
     * Make sure ball speed is above a minimum amount
     */
    bool velocityValidator(std::vector<WorldRobot>& robot, std::vector<WorldBall>& ball);

    /**
     * Make sure ball is moving away from robot that kicked it
     */
    bool distanceIncreasingValidator(std::vector<WorldRobot>& robot, std::vector<WorldBall>& ball);

    /**
     * Checks that the ball is being shot from the robot mouth
     */
    bool inFrontValidator(std::vector<WorldRobot>& robot, std::vector<WorldBall>& ball);

    std::deque<VisionState> stateHistory;

    static ConfigDouble* robot_dist_filter_cutoff;
    static ConfigDouble* one_robot_within_dist;
    static ConfigDouble* any_robot_past_dist;
    static ConfigDouble* min_ball_speed;
    static ConfigDouble* min_kick_dist;
    static ConfigDouble* max_kick_angle;
};