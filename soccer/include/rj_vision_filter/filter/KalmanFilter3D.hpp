#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <rj_vision_filter/filter/KalmanFilter.hpp>

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
    KalmanFilter3D(Geometry2d::Pose init_pose, Geometry2d::Twist init_twist);

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observation The observation for the current frame
     */
    void predict_with_update(Geometry2d::Pose observation);

    /**
     * @return Current position estimate
     */
    Geometry2d::Point get_pos() const;

    /**
     * @return Current heading angle estimate
     */
    double get_theta() const;

    /**
     * @return Current velocity estimate
     */
    Geometry2d::Point get_vel() const;

    /**
     * @return Current heading angle velocity estimate
     */
    double get_omega() const;

    /**
     * @return Current position covariance (X and Y)
     */
    Geometry2d::Point get_pos_cov() const;

    /**
     * @return Current heading covariance
     */
    double get_theta_cov() const;

    /**
     * @return Current velocity covariance (X and Y)
     */
    Geometry2d::Point get_vel_cov() const;

    /**
     * @return Current heading angle velocity covariance
     */
    double get_omega_cov() const;
};
}  // namespace vision_filter