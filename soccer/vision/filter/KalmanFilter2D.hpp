#include "KalmanFilter.hpp"
#include <Geometry2d/Point.hpp>

class KalmanFilter2D : public KalmanFilter {
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
}