#pragma once

#include <Geometry2d/Point.hpp>
#include <rj_vision_filter/filter/kalman_filter.hpp>

namespace vision_filter {
class KalmanFilter2D : public KalmanFilter {
public:
    /**
     * Creates a kalman filter with all the parameters set to 0 (F_k etc)
     */
    KalmanFilter2D();

    /**
     * Creates and initializes a kalman filter
     *
     * @param init_pos initial position
     * @param init_vel initial velocity
     */
    KalmanFilter2D(Geometry2d::Point init_pos, Geometry2d::Point init_vel);

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observation The position observation for the current frame
     */
    void predict_with_update(Geometry2d::Point observation);

    /**
     * @return Current position estimate
     */
    Geometry2d::Point get_pos() const;

    /**
     * @return Current velocity estimate
     */
    Geometry2d::Point get_vel() const;

    /**
     * @return Current position covariance (X and Y)
     */
    Geometry2d::Point get_pos_cov() const;

    /**
     * @return Current velocity covariance (X and Y)
     */
    Geometry2d::Point get_vel_cov() const;

    /**
     * Set's state velocity given XY velocity
     *
     * @param new_vel New velocity to use
     */
    void set_vel(Geometry2d::Point new_vel);
};
}  // namespace vision_filter