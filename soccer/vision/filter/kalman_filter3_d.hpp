#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include "vision/filter/kalman_filter.hpp"

namespace vision_filter {
class KalmanFilter3D : public KalmanFilter {
public:
    /**
     * Creates a kalman filter with all the parameters set to 0 (F_k etc)
     */
    KalmanFilter3D();

    /**
     * Creates and initializes a kalman filter
     *
     * @param init_pose initial pose
     * @param init_twist initial twist
     */
    KalmanFilter3D(rj_geometry::Pose init_pose, rj_geometry::Twist init_twist);

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observation The observation for the current frame
     */
    void predict_with_update(rj_geometry::Pose observation);

    /**
     * @return Current position estimate
     */
    rj_geometry::Point get_pos() const;

    /**
     * @return Current heading angle estimate
     */
    double get_theta() const;

    /**
     * @return Current velocity estimate
     */
    rj_geometry::Point get_vel() const;

    /**
     * @return Current heading angle velocity estimate
     */
    double get_omega() const;

    /**
     * @return Current position covariance (X and Y)
     */
    rj_geometry::Point get_pos_cov() const;

    /**
     * @return Current heading covariance
     */
    double get_theta_cov() const;

    /**
     * @return Current velocity covariance (X and Y)
     */
    rj_geometry::Point get_vel_cov() const;

    /**
     * @return Current heading angle velocity covariance
     */
    double get_omega_cov() const;
};
}  // namespace vision_filter