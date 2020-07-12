#include <rj_vision_filter/robot/KalmanRobot.hpp>

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <iostream>
#include <rj_vision_filter/params.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, kalman_robot, max_time_outside_vision, 0.5,
    "Max number of seconds without a measurement before the object is deleted")

KalmanRobot::KalmanRobot(unsigned int cameraID, RJ::Time creationTime,
                         CameraRobot initMeasurement,
                         const WorldRobot& previousWorldRobot)
    : cameraID(cameraID),
      health(filter::health::PARAM_init),
      lastUpdateTime(creationTime),
      lastPredictTime(creationTime),
      unwrapThetaCtr(0),
      robotID(initMeasurement.getRobotID()),
      previousMeasurements(kick::detector::PARAM_slow_kick_hist_length) {
    Geometry2d::Pose initPose = initMeasurement.getPose();
    Geometry2d::Twist initTwist(0, 0, 0);

    if (previousWorldRobot.getIsValid()) {
        initTwist.linear() = previousWorldRobot.getVel();
        initTwist.angular() = previousWorldRobot.getOmega();
    }

    filter = KalmanFilter3D(initPose, initTwist);

    previousMeasurements.push_back(initMeasurement);
    previousTheta = initTwist.angular();
}

void KalmanRobot::predict(RJ::Time currentTime) {
    lastPredictTime = currentTime;

    // Decrement but make sure you don't go too low
    health =
        std::max(health - filter::health::PARAM_dec, filter::health::PARAM_min);

    filter.predict();
}

void KalmanRobot::predictAndUpdate(RJ::Time currentTime,
                                   CameraRobot updateRobot) {
    lastPredictTime = currentTime;
    lastUpdateTime = currentTime;

    // Increment but make sure you don't go too high
    health =
        std::min(health + filter::health::PARAM_inc, filter::health::PARAM_max);

    // Keep last X camera observations in list for kick detection and filtering
    previousMeasurements.push_back(updateRobot);

    // Unwrap theta so we have a continuous heading
    double curTheta = updateRobot.getTheta();

    // See if it went below -pi
    // Note: PI/2 is used to give a good buffer on either side
    if (previousTheta < -M_PI_2 && curTheta > M_PI_2) {
        unwrapThetaCtr--;
        // Went above pi
    } else if (previousTheta > M_PI_2 && curTheta < -M_PI_2) {
        unwrapThetaCtr++;
    }

    previousTheta = curTheta;

    filter.predictWithUpdate(
        {updateRobot.getPos(), curTheta + unwrapThetaCtr * 2 * M_PI});
}

bool KalmanRobot::isUnhealthy() const {
    bool updated_recently = RJ::Seconds(lastPredictTime - lastUpdateTime) <
                            RJ::Seconds(*max_time_outside_vision);

    return !updated_recently;
}

unsigned int KalmanRobot::getCameraID() const { return cameraID; }

int KalmanRobot::getRobotID() const { return robotID; }

int KalmanRobot::getHealth() const { return health; }

Geometry2d::Point KalmanRobot::getPos() const { return filter.getPos(); }

double KalmanRobot::getTheta() const { return filter.getTheta(); }

Geometry2d::Point KalmanRobot::getVel() const { return filter.getVel(); }

double KalmanRobot::getOmega() const { return filter.getOmega(); }

Geometry2d::Point KalmanRobot::getPosCov() const { return filter.getPosCov(); }

double KalmanRobot::getThetaCov() const { return filter.getThetaCov(); }

Geometry2d::Point KalmanRobot::getVelCov() const { return filter.getVelCov(); }

double KalmanRobot::getOmegaCov() const { return filter.getOmegaCov(); }

const boost::circular_buffer<CameraRobot>& KalmanRobot::getPrevMeasurements()
    const {
    return previousMeasurements;
}
}  // namespace vision_filter