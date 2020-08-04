#pragma once

#include <Geometry2d/Point.hpp>
#include <rj_vision_filter/ball/KalmanBall.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>
#include <vector>

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
     * @param yellowRobots Best estimation of the yellow robots states
     * @param blueRobots Best estimation of the yellow robots states
     * @param outNewVel Output of the resulting velocity vector after bounce
     *
     * @return Whether the ball bounces or not
     */
    static bool CalcBallBounce(const KalmanBall& ball,
                               const std::vector<WorldRobot>& yellowRobots,
                               const std::vector<WorldRobot>& blueRobots,
                               Geometry2d::Point& outNewVel);

private:
    /**
     * Returns whether the ball is most likely intersecting the robots
     *
     * Note: Ignores the extra mouth calculations
     *
     * @param ball The ball we want to check for intersection with
     * @param robot The robot we what to check for intersection with
     */
    static bool BallInRobot(const KalmanBall& ball, const WorldRobot& robot);

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
    static std::vector<Geometry2d::Point> PossibleBallIntersectionPts(
        const KalmanBall& ball, const WorldRobot& robot);
};
}  // namespace vision_filter