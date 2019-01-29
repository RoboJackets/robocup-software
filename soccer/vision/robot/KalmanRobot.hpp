#pragma once

#include <boost/circular_buffer.hpp>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <Configuration.hpp>

#include "vision/robot/CameraRobot.hpp"
#include "vision/filter/KalmanFilter3D.hpp"

class WorldRobot;

/**
 * Filtered robot estimation for a single camera
 */
class KalmanRobot {
public:
    /**
     * Checks previousWorldRobot to see if it's valid
     *
     * @param cameraID ID of the camera this filter is applied to
     * @param creationTime Time this filter was created
     * @param initMeasurement Initial robot measurement
     * @param previousWorldRobot World robot from last frame (or invalid world robot)
     */
    KalmanRobot(unsigned int cameraID, RJ::Time creationTime,
                CameraRobot initMeasurement, const WorldRobot& previousWorldRobot);

    /**
     * Predicts one time step forward
     *
     * @param currentTime Current time of the prediction step
     */
    void predict(RJ::Time currentTime);

    /**
     * Predicts one time step forward then triangulates toward the measurement
     *
     * @param currentTime Current time of the prediction/update step
     * @param updateRobot Robot measurement that we are using as feedback
     */
    void predictAndUpdate(RJ::Time currentTime, CameraRobot updateRobot);

    /**
     * Returns true when the filter hasn't been updated in a while and should be deleted
     */
    bool isUnhealthy() const;

    /**
     * @return The camera id this belongs to
     */
    unsigned int getCameraID() const;

    /**
     * @return This robot's id
     */
    int getRobotID() const;

    /**
     * @return How healthy this filter is. AKA How often it's been updated
     */
    int getHealth() const;

    /**
     * @return Best estimate of the linear position of the robot
     */
    Geometry2d::Point getPos() const;

    /**
     * @return Best estimate of the heading. Not bounded
     */
    double getTheta() const;

    /**
     * @return Best estimate of the linear velocity of the robot
     */
    Geometry2d::Point getVel() const;

    /**
     * @return Best estimate of the angular velocity
     */
    double getOmega() const;

    /**
     * @return Covariance in X and Y linear direction of the position of the robot
     */
    Geometry2d::Point getPosCov() const;

    /**
     * @return Covariance of theta of the robot
     */
    double getThetaCov() const;

    /**
     * @return Covariance in X and Y linear direction of the velocity of the robot
     */
    Geometry2d::Point getVelCov() const;

    /**
     * @return Covariance of omega of the robot
     */
    double getOmegaCov() const;

    /**
     * @return List of previous camera robot measurements for kick detection
     */
    const boost::circular_buffer<CameraRobot>& getPrevMeasurements() const;

    static void createConfiguration(Configuration* cfg);

private:
    RJ::Time lastUpdateTime;
    RJ::Time lastPredictTime;

    boost::circular_buffer<CameraRobot> previousMeasurements;

    KalmanFilter3D filter;

    double previousTheta;
    int unwrapThetaCtr;
    int health;

    int robotID;

    unsigned int cameraID;

    // Max number of seconds without a measurement before the object
    // is deleted
    static ConfigDouble* max_time_outside_vision;
};