#pragma once

#include <deque>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <Configuration.hpp>

#include "vision/ball/CameraBall.hpp"
#include "vision/ball/WorldBall.hpp"
#include "vision/filter/KalmanFilter2D.hpp"

/**
 * Filtered ball estimation for a single camera
 */
class KalmanBall {
public:
    /**
     * Checks the previousWorldBall to see if it's valid
     *
     * @param cameraId ID of the camera this filter belongs to
     * @param creationTime Time this filter is created
     * @param initMeasurement Initial ball measurement we are creating the filter at
     * @param previousWorldBall Previous prediction of ball location to initialize the velocity smartly
     */
    KalmanBall(unsigned int cameraID, RJ::Time creationTime,
               CameraBall initMeasurement, WorldBall& previousWorldBall);

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

    unsigned int getCameraID();

    int getHealth();

    Geometry2d::Point getPos();
    Geometry2d::Point getVel();
    Geometry2d::Point getPosCov();
    Geometry2d::Point getVelCov();

    std::deque<CameraBall> getPrevMeasurements();

    /**
     * Note: Only used to set the velocity when we think the ball will bounce off another robot
     */
    void setVel(Geometry2d::Point newVel);

    static void createConfiguration(Configuration* cfg);

private:
    ConfigDouble* max_time_outside_vision;

    RJ::Time lastUpdateTime;
    RJ::Time lastPredictTime;

    // Keeps track of this for kick detection stuff
    std::deque<CameraBall> previousMeasurements;

    KalmanFilter2D filter;

    int health;

    unsigned int cameraID;
};