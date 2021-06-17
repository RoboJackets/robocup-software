#pragma once

#include <list>

#include <rj_geometry/point.hpp>

#include "vision/ball/kalman_ball.hpp"

namespace vision_filter {
class KalmanBall;

/**
 * Best estimate of the true ball position using the kalman balls from each
 * camera
 */
class WorldBall {
public:
    /**
     * Creates an invalid world ball
     * Used to make some of the code cleaner
     */
    WorldBall();

    /**
     * Creates a valid world ball from a list of kalman balls through special
     * averaging
     *
     * @param calc_time Current iteration time
     * @param kalmanBalls List of best kalman ball from every camera
     */
    WorldBall(RJ::Time calc_time, const std::list<KalmanBall>& kalman_balls);

    /**
     * @return If the ball actually represents a real ball
     */
    bool get_is_valid() const;

    /**
     * @return The best estimated position of the ball
     */
    rj_geometry::Point get_pos() const;

    /**
     * @return The best estimated velocity of the ball
     */
    rj_geometry::Point get_vel() const;

    /**
     * @return The average position covariance of the filter
     */
    double get_pos_cov() const;

    /**
     * @return The average velocity covariance of the filter
     */
    double get_vel_cov() const;

    /**
     * @return List of all the building kalman balls for this world ball
     */
    const std::list<KalmanBall>& get_ball_components() const;

    /**
     * @return Time of creation for this world ball
     */
    RJ::Time get_time() const;

private:
    bool is_valid_;
    rj_geometry::Point pos_;
    rj_geometry::Point vel_;
    double pos_cov_{};
    double vel_cov_{};
    std::list<KalmanBall> ball_components_;
    RJ::Time time_;
};
}  // namespace vision_filter
