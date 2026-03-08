#pragma once

#include <rj_geometry/point.hpp>
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
    KalmanFilter2D(
        rj_geometry::Point init_pos,
        rj_geometry::Point init_vel,
        double vision_loop_dt = 1.0 / 60.0,
        double ball_initial_covariance = 100.0,
        double ball_process_noise = 0.1,
        double ball_observation_noise = 2.0
    );

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observation The position observation for the current frame
     */
    void predict_with_update(rj_geometry::Point observation);

    /**
     * @return Current position estimate
     */
    [[nodiscard]] rj_geometry::Point get_pos() const;

    /**
     * @return Current velocity estimate
     */
    [[nodiscard]] rj_geometry::Point get_vel() const;

    /**
     * @return Current position covariance (X and Y)
     */
    [[nodiscard]] rj_geometry::Point get_pos_cov() const;

    /**
     * @return Current velocity covariance (X and Y)
     */
    [[nodiscard]] rj_geometry::Point get_vel_cov() const;

    /**
     * Set's state velocity given XY velocity
     *
     * @param new_vel New velocity to use
     */
    void set_vel(rj_geometry::Point new_vel);
};
}  // namespace vision_filter