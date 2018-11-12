#include <vector>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <vector>

#include "vision/ball/CameraBall.hpp"
#include "vision/ball/WorldBall.hpp"
#include "vision/filter/KalmanFilter2D.hpp"

class KalmanBall {
public:
    /**
     * @param cameraId ID of the camera this filter belongs to
     * @param creationTime Time this filter is created
     * @param initPos Initial ball measurement we are creating the filter at
     * @param previousWorldBall Previous prediction of ball location to initialize the velocity smartly
     */
    KalmanBall(unsigned int cameraId, RJ::Time creationTime, CameraBall initPos, WorldBall previousWorldBall);

    /**
     * Predicts one time step forward
     */
    void predict(RJ::Time currentTime);

    /**
     * Predicts one time step forward then triangulates towards the measurement
     *
     * @param currentTime Current time of the prediction/update step
     * @param updateBall Ball measurement that we are using as feedback to the filters
     */
    void predictAndUpdate(RJ::Time currentTime, CameraBall updateBall);

    /**
     * Returns true when the filter hasn't been updated in a while etc and should be deleted
     */
    bool isUnhealthy();

    Geometry2d::Point getPos();
    Geometry2d::Point getVel();

    /**
     * Note: Only used to set the velocity when we think the ball will bounce off another robot
     */
    void setVel(Geometry2d::Point newVel);

private:
    RJ::Time lastUpdateTime;
    RJ::Time lastPredictTime;

    // Keeps track of this for kick detection stuff
    std::vector<CameraBall> previousMeasurements;

    KalmnaFilter2D filter;

    unsigned int cameraId;
};