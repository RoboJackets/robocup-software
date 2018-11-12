#pragma once

#include "KalmanFilter.hpp"
#include <Geometry2d/Point.hpp>

class KalmanFilter2D : public KalmanFilter {
public:
    KalmanFilter2D(Geometry2d::Point initPos, Geometry2d::Point initVel);

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observation The position observation for the current frame
     */
    void PredictWithUpdate(Geometry2d::Point observation);

    Geometry2d::Point getPos();
    Geoemtry2d::Point getVel();

    Geometry2d::Point getPosCov();
    Geometry2d::Point getVelCov();

    void setVel(Geometry2d::Point newVel);

    static void createConfiguation(Configuration* cfg);

private:
    static ConfigDouble* ball_init_covariance;
    static ConfigDouble* ball_process_noise;
    static ConfigDouble* ball_observation_noise;
}