#include "KalmanRobot.hpp"

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#include "vision/util/VisionFilterConfig.hpp"

void KalmanRobot::createConfiguration(Configuration* cfg) {
    max_time_outside_vision = new ConfigDouble("VisionFilter/KalmanRobot/max_time_outside_vision", 2.0);
}

KalmanRobot::KalmanRobot(unsigned int cameraID, RJ::Time creationTime,
                         CameraRobot initMeasurement)
    : cameraID(cameraID), health(*VisionFilterConfig::filter_health_init),
      lastUpdateTime(creationTime), lastPredictTime(creationTime),
      unwrapThetaCtr(0), robotID(initMeasurement.getRobotID()) {

    Geometry2d::Point initPos = initMeasurement.getPos();
    double initTheta = initMeasurement.getTheta();
    Geometry2d::Point initVel = Geometry2d::Point(0,0);
    double initOmega = 0.0;

    filter = KalmanFilter3D(initPos, initTheta, initVel, initOmega);

    previousMeasurements.push_back(initMeasurement);
}

KalmanRobot::KalmanRobot(unsigned int cameraID, RJ::Time creationTime,
                         CameraRobot initMeasurement, WorldRobot& previousWorldRobot)
    : cameraID(cameraID), health(*VisionFilterConfig::filter_health_init),
      lastUpdateTime(creationTime), lastPredictTime(creationTime),
      unwrapThetaCtr(0), robotID(initMeasurement.getRobotID()) {

    Geometry2d::Point initPos = initMeasurement.getPos();
    double initTheta = initMeasurement.getTheta();
    Geometry2d::Point initVel = previousWorldRobot.getVel();
    double initOmega = previousWorldRobot.getOmega();

    filter = KalmanFilter3D(initPos, initTheta, initVel, initOmega);

    previousMeasurements.push_back(initMeasurement);
}

void KalmanRobot::predict(RJ::Time currentTime) {
    lastPredictTime = currentTime;

    // Decrement but make sure you don't go too low
    health = std::max(health - *VisionFilterConfig::filter_health_dec,
                      *VisionFilterConfig::filter_health_min);

    filter.predict();
}

void KalmanRobot::predictAndUpdate(RJ::Time currentTime, CameraRobot updateRobot) {
    lastPredictTime = currentTime;
    lastUpdateTime = currentTime;

    // Increment but make sure you don't go too high
    health = std::min(health + *VisionFilterConfig::filter_health_inc,
                      *VisionFilterConfig::filter_health_max);

    // Keep last X camera observations in list for kick detection and filtering
    previousMeasurements.push_back(initMeasurement);
    if (previousMeasurements.size() > *VisionFilterConfig::slow_kick_detector_history_length) {
        previousMeasurements.pop_front();
    }

    // Unwrap theta so we have a continuous heading
    prevTheta = *previousMeasurements.end();
    curTheta = updateRobot.getTheta();

    // See if it went below -pi
    // Note: PI/2 is used to give a good buffer on either side
    if (prevTheta < -M_PI_2 && curTheta > M_PI_2) {
        unwrapThetaCtr--;
    // Went above pi
    } else if (prevTheta > M_PI_2 && curTheta < -M_PI_2) {
        unwrapThetaCtr++;
    }

    filter.PredictWithUpdate(updateRobot.getPos(), curTheta + unwrapThetaCtr*2*M_PI);
}

bool KalmanRobot::isUnhealthy() {
    updated_recently = RJ::numSeconds(lastPredictTime - lastUpdatedTime) < *max_time_outside_vision;

    return !updated_recently;
}

unsigned int KalmanRobot::getCameraID() {
    return cameraID;
}

Geometry2d::Point KalmanRobot::getPos() {
    return filter.getPos();
}

double KalmanRobot::getTheta() {
    return filter.getTheta();
}

Geometry2d::Point KalmanRobot::getVel() {
    return filter.getVel();
}

double KalmnaRobot::getOmega() {
    return fitler.getOmega();
}