#pragma once

#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include <deque>
#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/kick/KickEvent.hpp>
#include <rj_vision_filter/kick/VisionState.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>

/**
 * Accurately detects kicks by robots using 5 or more samples
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
    bool addRecord(RJ::Time calcTime, const WorldBall& ball,
                   const std::vector<WorldRobot>& yellowRobots,
                   const std::vector<WorldRobot>& blueRobots,
                   KickEvent& kickEvent);

    static void createConfiguration(Configuration* cfg);

private:
    /**
     * Tries to find out if/which robot kicked
     *
     * @param kickEvent Returned kick event if one is detected
     *
     * @return whether a kick event was detected
     */
    bool detectKick(KickEvent& kickEvent);

    /**
     * Checks to see if all the different tests to detect kicks are true
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    bool checkAllValidators(std::vector<WorldRobot>& robot, std::vector<WorldBall>& ball);

    /**
     * If ball and robots were close and are now far away
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool distanceValidator(std::vector<WorldRobot>& robot,
                                  std::vector<WorldBall>& ball);

    /**
     * Make sure ball speed is above a minimum amount
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool velocityValidator(std::vector<WorldRobot>& robot,
                                  std::vector<WorldBall>& ball);

    /**
     * Make sure ball is moving away from robot that kicked it
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool distanceIncreasingValidator(std::vector<WorldRobot>& robot,
                                            std::vector<WorldBall>& ball);

    /**
     * Checks that the ball is being shot from the robot mouth
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool inFrontValidator(std::vector<WorldRobot>& robot,
                                 std::vector<WorldBall>& ball);

    std::deque<VisionState> stateHistory;

    // Doesn't check any robots past this distance for optimization
    static ConfigDouble* robot_dist_filter_cutoff;
    // Only one ball measurement within this distance of the robot
    static ConfigDouble* one_robot_within_dist;
    // At least one ball measurement past this distance of the robot
    static ConfigDouble* any_robot_past_dist;
    // Ball has to be this fast
    static ConfigDouble* min_ball_speed;
    // Max angle difference between velocity vector and robot heading
    static ConfigDouble* max_kick_angle;
};
