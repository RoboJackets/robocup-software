#pragma once

#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include <boost/circular_buffer.hpp>
#include <rj_common/Utils.hpp>

#include <rj_vision_filter/ball/CameraBall.hpp>
#include <rj_vision_filter/filter/KalmanFilter2D.hpp>

class WorldBall;

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
               CameraBall initMeasurement, const WorldBall& previousWorldBall);

    /**
     * Predicts one time step forward
     *
     * @param currentTime Time at the current frame
     *
     * @note Call either this OR predictAndUpdate once a frame
     */
    void predict(RJ::Time currentTime);

    /**
     * Predicts one time step forward then triangulates towards the measurement
     *
     * @param currentTime Current time of the prediction/update step
     * @param updateBall Ball measurement that we are using as feedback to the filters
     *
     * @note Call either this OR predict once a frame
     */
    void predictAndUpdate(RJ::Time currentTime, CameraBall updateBall);

    /**
     * @return Returns true when the filter hasn't been updated in a while etc and should be deleted
     */
    bool isUnhealthy() const;

    /**
     * @return The camera id this belongs to
     */
    unsigned int getCameraID() const;

    /**
     * @return How healthy this filter is. AKA How often it's been updated
     */
    int getHealth() const;

    /**
     * @return Best estimate of the position of the ball
     */
    Geometry2d::Point getPos() const;

    /**
     * @return Best estimate of the velocity of the ball
     */
    Geometry2d::Point getVel() const;

    /**
     * @return Covariance in X and Y direction of the position of the ball
     */
    Geometry2d::Point getPosCov() const;

    /**
     * @return Covariance in X and Y direction of the velocity of the ball
     */
    Geometry2d::Point getVelCov() const;

    /**
     * @return List of previous camera ball measurements for kick detection/estimation
     */
    const boost::circular_buffer<CameraBall>& getPrevMeasurements() const;

    /**
     * @param newVel new velocity to insert into the kalman filter
     *
     * Note: Only used to set the velocity when we think the ball will bounce
     * off another robot
     */
    void setVel(Geometry2d::Point newVel);

    static void createConfiguration(Configuration* cfg);

private:
    RJ::Time lastUpdateTime;
    RJ::Time lastPredictTime;

    // Keeps track of this for kick detection stuff
    boost::circular_buffer<CameraBall> previousMeasurements;

    KalmanFilter2D filter;

    int health;

    unsigned int cameraID;

    // Max time in seconds that a filter can not be updated before it is removed
    static ConfigDouble* max_time_outside_vision;
};
