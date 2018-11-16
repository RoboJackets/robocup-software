#pragma once

#include <deque>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <Configuration.hpp>

#include "vision/robot/CameraRobot.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "vision/filter/Kalmanfilter3D.hpp"

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
     */
    KalmanRobot(unsigned int cameraID, RJ::Time creationTime,
                CameraRobot initMeasurement, WorldRobot& previousWorldRobot);

    /**
     * Predicts one time step forward
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
     * Returns true when the filter hasn't been updated in a while and should be deteled
     */
    bool isUnhealthy();

    unsigned int getCameraID();

    Geometry2d::Point getPos();
    double getTheta();
    Geometry2d::Point getVel();
    double getOmega();

    int getRobotID();

    std::deque<CameraRobot> getPrevMeasurements();

    static void createConfiguration(Configuration* cfg);

private:
    ConfigDouble* max_time_outside_vision;

    RJ::Time lastUpdateTime;
    RJ::Time lastPredictTime;

    std::deque<CameraRobot> previousMeasurements;

    KalmanFilter3D filter;

    int unwrapThetaCtr;
    int robotID;
    int health;

    unsigned int cameraID;
};