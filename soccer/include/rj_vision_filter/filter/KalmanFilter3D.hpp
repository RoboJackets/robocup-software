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
     * @param initPose initial pose
     * @param initTwist initial twist
     */
    KalmanFilter3D(Geometry2d::Pose initPose, Geometry2d::Twist initTwist);

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observation The observation for the current frame
     */
    void predictWithUpdate(Geometry2d::Pose observation);

    /**
     * @return Current position estimate
     */
    Geometry2d::Point getPos() const;

    /**
     * @return Current heading angle estimate
     */
    double getTheta() const;

    /**
     * @return Current velocity estimate
     */
    Geometry2d::Point getVel() const;

    /**
     * @return Current heading angle velocity estimate
     */
    double getOmega() const;

    /**
     * @return Current position covariance (X and Y)
     */
    Geometry2d::Point getPosCov() const;

    /**
     * @return Current heading covariance
     */
    double getThetaCov() const;

    /**
     * @return Current velocity covariance (X and Y)
     */
    Geometry2d::Point getVelCov() const;

    /**
     * @return Current heading angle velocity covariance
     */
    double getOmegaCov() const;
};
}  // namespace vision_filter