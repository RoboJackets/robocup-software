#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_geometry/point.hpp>
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
    SlowKickDetector(const std::shared_ptr<rclcpp::Node>& vision_filter_node);
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
    [[nodiscard]] bool detect_kick(KickEvent* kick_event) const;

    /**
     * Checks to see if all the different tests to detect kicks are true
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    [[nodiscard]] bool check_all_validators(const std::vector<WorldRobot>& robot,
                                   const std::vector<WorldBall>& ball) const;

    /**
     * If ball and robots were close and are now far away
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    [[nodiscard]] bool distance_validator(const std::vector<WorldRobot>& robot,
                                  const std::vector<WorldBall>& ball) const;

    /**
     * Make sure ball speed is above a minimum amount
     *
     * @param robot List of robots as a function of time to check against
     * @param ball List of balls as a function of time to check against
     *
     * @note robots and balls should be time synced
     */
    [[nodiscard]] bool velocity_validator(const std::vector<WorldRobot>& robot,
                                  const std::vector<WorldBall>& ball) const;

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
    [[nodiscard]] bool in_front_validator(const std::vector<WorldRobot>& robot,
                                 const std::vector<WorldBall>& ball) const;

    std::deque<VisionState> state_history_;

    // Doesn't check any robots past this distance in meters for optimization
    double slow_robot_dist_filter_cutoff_ = 3.0;
    // Only one ball measurement within this distance of the robot
    double slow_one_robot_within_dist_ = 0.15;
    // At least one ball measurement past this distance of the robot
    double slow_any_robot_past_dist_ = 0.16;
    // Ball has to be this fast
    double slow_min_ball_speed_ = 0.6;
    // Max angle difference between velocity vector and robot heading
    double slow_max_kick_angle_ = 0.34;
    // The length of the slow kick history buffer
    int slow_kick_hist_length_ = 5;
    // The length of the fast kick history buffer
    int fast_kick_hist_length_ = 3;
    // The time between vision loops
    double vision_loop_dt_ = 1.0 / 60.0;

    // Callback to update the ros parameters for the filter
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;
};
}  // namespace vision_filter
