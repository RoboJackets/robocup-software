#pragma once

#include "KalmanFilter.hpp"
#include <Geometry2d/Point.hpp>

class KalmanFilter3D : public KalmanFilter {
    KalmanFilter3D(Geometry2d::Point initPos, double theta,
                   Geometry2d::Point initVel, double omega);

    /**
     * Predicts with update
     * Overrides the standard PredictWithUpdate and sets the z_k automatically
     *
     * @param observationPos The position observation for the current frame
     * @param observationTheta The theta observation for the current frame
     */
    void PredictWithUpdate(Geometry2d::Point observationPos, double observationTheta);

    Geometry2d::Point getPos();
    double getTheta();

    Geometry2d::Point getVel();
    double getOmega();

    Geometry2d::Point getPosCov();
    double getThetaCov();

    Geometry2d::Point getVelCov();
    double getOmegaCov();

    static void createConfiguation(Configuration* cfg);

private:
    static ConfigDouble* robot_init_covariance;
    static ConfigDouble* robot_process_noise;
    static ConfigDouble* robot_observation_noise;
}