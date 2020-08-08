#pragma once

#include <Geometry2d/Point.hpp>
#include <deque>
#include <rj_vision_filter/ball/world_ball.hpp>
#include <rj_vision_filter/kick/kick_event.hpp>
#include <rj_vision_filter/kick/vision_state.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>

namespace vision_filter {
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
     * @param calc_time Time of calculation for this vision loop
     * @param ball Best estimation of the current ball
     * @param yellow_robots Best estimation of the yellow robots
     * @param blue_robots Best estimation of the blue robots
     * @param kick_event Returned kick event if we find one
     *
     * @return Whether there was a kick
     *
     * @note kick_event is only filled if it returns true
     * It will change, but will have invalid data in it
     */
    bool add_record(RJ::Time calc_time, const WorldBall& ball,
                   const std::vector<WorldRobot>& yellow_robots,
                   const std::vector<WorldRobot>& blue_robots,
                   KickEvent* kick_event);

private:
    /**
     * Tries to find out if/which robot kicked
     *
     * @param kick_event Returned kick event if one is detected
     *
     * @return whether a kick event was detected
     */
    bool detect_kick(KickEvent* kick_event);

    /**
     * Checks to see if all the different tests to detect kicks are true
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool check_all_validators(const std::vector<WorldRobot>& robot,
                                   const std::vector<WorldBall>& ball);

    /**
     * If ball and robots were close and are now far away
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool distance_validator(const std::vector<WorldRobot>& robot,
                                  const std::vector<WorldBall>& ball);

    /**
     * Make sure ball speed is above a minimum amount
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool velocity_validator(const std::vector<WorldRobot>& robot,
                                  const std::vector<WorldBall>& ball);

    /**
     * Make sure ball is moving away from robot that kicked it
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool distance_increasing_validator(
        const std::vector<WorldRobot>& robot,
        const std::vector<WorldBall>& ball);

    /**
     * Checks that the ball is being shot from the robot mouth
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    static bool in_front_validator(const std::vector<WorldRobot>& robot,
                                 const std::vector<WorldBall>& ball);

    std::deque<VisionState> state_history_;
};
}  // namespace vision_filter
