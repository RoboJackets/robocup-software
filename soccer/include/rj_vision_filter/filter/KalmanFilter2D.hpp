#pragma once

#include <rj_vision_filter/filter/KalmanFilter.hpp>
#include <Geometry2d/Point.hpp>
#include <Configuration.hpp>

class KalmanFilter2D : public KalmanFilter {
public:
    /**
     * Creates a kalman filter with all the parameters set to 0 (F_k etc)
     */
    KalmanFilter2D();

    /**
     * Creates and initializes a kalman filter
     *
     * @param initPos initial position
     * @param initVel initial velocity
     */
    KalmanFilter2D(Geometry2d::Point initPos, Geometry2d::Point initVel);

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observation The position observation for the current frame
     */
    void predictWithUpdate(Geometry2d::Point observation);

    /**
     * @return Current position estimate
     */
    Geometry2d::Point getPos() const;

    /**
     * @return Current velocity estimate
     */
    Geometry2d::Point getVel() const;

    /**
     * @return Current position covariance (X and Y)
     */
    Geometry2d::Point getPosCov() const;

    /**
     * @return Current velocity covariance (X and Y)
     */
    Geometry2d::Point getVelCov() const;

    /**
     * Set's state velocity given XY velocity
     * 
     * @param newVel New velocity to use
     */
    void setVel(Geometry2d::Point newVel);

    static void createConfiguration(Configuration* cfg);

private:
    // Initial covariance of the filter
    // Controls how fast it gets to target
    static ConfigDouble* ball_init_covariance;
    // Controls how quickly it reacts to changes in ball accelerations
    static ConfigDouble* ball_process_noise;
    // Controls how much it trusts measurements from the camera
    static ConfigDouble* ball_observation_noise;
};