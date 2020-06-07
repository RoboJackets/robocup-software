#include "KalmanBall.hpp"

#include <algorithm>

#include "vision/ball/WorldBall.hpp"
#include "vision/util/VisionFilterConfig.hpp"

REGISTER_CONFIGURABLE(KalmanBall)

ConfigDouble* KalmanBall::max_time_outside_vision;

void KalmanBall::createConfiguration(Configuration* cfg) {
    max_time_outside_vision = new ConfigDouble(cfg, "VisionFilter/KalmanBall/max_time_outside_vision", 0.2);
}

KalmanBall::KalmanBall(unsigned int cameraID, RJ::Time creationTime,
                       CameraBall initMeasurement, const WorldBall& previousWorldBall)
    : lastUpdateTime(creationTime), lastPredictTime(creationTime),
      previousMeasurements(*VisionFilterConfig::slow_kick_detector_history_length),
      health(*VisionFilterConfig::filter_health_init), cameraID(cameraID) {
  geometry2d::Point initPos = initMeasurement.getPos();
  geometry2d::Point initVel = geometry2d::Point(0, 0);

  // If we have a world ball, use that vel as init to smooth cam transitions
  if (previousWorldBall.getIsValid()) {
    initVel = previousWorldBall.getVel();
    }

    filter = KalmanFilter2D(initPos, initVel);

    previousMeasurements.push_back(initMeasurement);
}

void KalmanBall::predict(RJ::Time currentTime) {
    lastPredictTime = currentTime;

    // Decrement but make sure you don't go too low
    health = std::max(health - *VisionFilterConfig::filter_health_dec,
                      static_cast<int>(*VisionFilterConfig::filter_health_min));

    filter.predict();
}

void KalmanBall::predictAndUpdate(RJ::Time currentTime, CameraBall updateBall) {
    lastPredictTime = currentTime;
    lastUpdateTime = currentTime;

    // Increment but make sure you don't go too high
    health = std::min(health + *VisionFilterConfig::filter_health_inc,
                      static_cast<int>(*VisionFilterConfig::filter_health_max));

    // Keep last X camera observations in list for kick detection and filtering
    previousMeasurements.push_back(updateBall);

    filter.predictWithUpdate(updateBall.getPos());
}

bool KalmanBall::isUnhealthy() const {
    bool updated_recently = RJ::Seconds(lastPredictTime - lastUpdateTime) < RJ::Seconds(*max_time_outside_vision);

    return !updated_recently;
}

unsigned int KalmanBall::getCameraID() const {
    return cameraID;
}

int KalmanBall::getHealth() const {
    return health;
}

geometry2d::Point KalmanBall::getPos() const { return filter.getPos(); }

geometry2d::Point KalmanBall::getVel() const { return filter.getVel(); }

geometry2d::Point KalmanBall::getPosCov() const { return filter.getPosCov(); }

geometry2d::Point KalmanBall::getVelCov() const { return filter.getVelCov(); }

const boost::circular_buffer<CameraBall>& KalmanBall::getPrevMeasurements() const {
    return previousMeasurements;
}

void KalmanBall::setVel(geometry2d::Point newVel) { filter.setVel(newVel); }
