#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_geometry/point.hpp>
#include <rj_vision_filter/ball/kalman_ball.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>
#include <vector>

namespace vision_filter {

//NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class BallBounce {
public:
    /**
     * These functions are wrapped into a class instead of a namespace so that
     * the config system can be used. It requires a class with the
     * REGISTER_CONFIGUABLE define. Additionally, this allows for the extra
     * helper functions to be hidden.
     */
    BallBounce() = default;

    BallBounce(const std::shared_ptr<rclcpp::Node>& vision_filter_node);

    /**
     * Calculates whether the given kalman ball will bounce against another
     * robot and the resulting velocity vector
     *
     * @param ball Kalman ball we are trying to test
     * @param yellow_robots Best estimation of the yellow robots states
     * @param blue_robots Best estimation of the yellow robots states
     * @param out_new_vel Output of the resulting velocity vector after bounce
     *
     * @return Whether the ball bounces or not
     */
    bool calc_ball_bounce(const KalmanBall& ball,
                               const std::vector<WorldRobot>& yellow_robots,
                               const std::vector<WorldRobot>& blue_robots,
                               rj_geometry::Point& out_new_vel);

private:
    /**
     * Returns whether the ball is most likely intersecting the robots
     *
     * Note: Ignores the extra mouth calculations
     *
     * @param ball The ball we want to check for intersection with
     * @param robot The robot we what to check for intersection with
     */
    [[nodiscard]] bool ball_in_robot(const KalmanBall& ball, const WorldRobot& robot) const;

    /**
     * Finds the 0, 1 or 2 interserct locations on the ball shell
     *
     * @param ball The ball we want to check for
     * @param robot The robot we want to check against
     *
     * @return List of all intersection points. Length 0, 1, or 2
     * 0 means no intersection
     * 1 means tangental intersection
     * 2 means chord based intersection
     */
    static std::vector<rj_geometry::Point> possible_ball_intersection_pts(
        const KalmanBall& ball, const WorldRobot& robot);

    /**
     * @brief Initialize the ros parameters used by this filter
     * 
     */
    void initialize_parameters(const std::shared_ptr<rclcpp::Node>& vision_filter_node);

    /**
     * @brief Update the stored ros parameters used by this filter
     * 
     * @param params 
     * @return true 
     * @return false 
     */
    bool update_parameters(const std::vector<rclcpp::Parameter>& params);

    // The linear velocity dampen for bouncing off the circular shell (1 means 100% of the velocity
    // is kept after a collision)
    double robot_body_linear_dampening_ = 0.9;
    // Linear velocity dampening for bouncing off the front mouth
    double robot_mouth_linear_dampening_ = 0.3;
    // Reflect angle dampening for bouncing off the circular shell
    double robot_body_angle_dampening_ = 0.0;
    // Reflect angle dampening for bouncing off the front mouth
    double robot_mouth_angle_dampening_ = 0.0;
    // The time between vision processing loops
    double vision_loop_dt_ = 1.0 / 60.0;

    // A reference to the parameter callback handle
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;
};
}  // namespace vision_filter