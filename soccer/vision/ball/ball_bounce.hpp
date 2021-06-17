#pragma once

#include <vector>

#include <rj_geometry/point.hpp>

#include "vision/ball/kalman_ball.hpp"
#include "vision/robot/world_robot.hpp"

namespace vision_filter {
class BallBounce {
public:
    /**
     * These functions are wrapped into a class instead of a namespace so that
     * the config system can be used. It requires a class with the
     * REGISTER_CONFIGUABLE define. Additionally, this allows for the extra
     * helper functions to be hidden.
     */
    BallBounce() = delete;
    ~BallBounce() = delete;

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
    static bool calc_ball_bounce(const KalmanBall& ball,
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
    static bool ball_in_robot(const KalmanBall& ball, const WorldRobot& robot);

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
    static std::vector<rj_geometry::Point> possible_ball_intersection_pts(const KalmanBall& ball,
                                                                          const WorldRobot& robot);
};
}  // namespace vision_filter