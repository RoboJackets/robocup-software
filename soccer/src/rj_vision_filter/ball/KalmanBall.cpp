#include <algorithm>

#include <rj_vision_filter/ball/KalmanBall.hpp>
#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, kalman_ball, max_time_outside_vision, 0.2,
                  "Max time in seconds that a filter can not be updated before it "
                  "is removed.")
using kalman_ball::PARAM_max_time_outside_vision;

KalmanBall::KalmanBall(unsigned int cameraID, RJ::Time creationTime, CameraBall initMeasurement,
                       const WorldBall& previousWorldBall)
    : lastUpdateTime(creationTime),
      lastPredictTime(creationTime),
      previousMeasurements(kick::detector::PARAM_slow_kick_hist_length),
      health(filter::health::PARAM_init),
      cameraID(cameraID) {
    Geometry2d::Point initPos = initMeasurement.getPos();
    Geometry2d::Point initVel = Geometry2d::Point(0, 0);

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
    health = std::max(health - filter::health::PARAM_dec, filter::health::PARAM_min);

    filter.predict();
}

void KalmanBall::predictAndUpdate(RJ::Time currentTime, CameraBall updateBall) {
    lastPredictTime = currentTime;
    lastUpdateTime = currentTime;

    // Increment but make sure you don't go too high
    health = std::min(health + filter::health::PARAM_inc, filter::health::PARAM_max);

    // Keep last X camera observations in list for kick detection and filtering
    previousMeasurements.push_back(updateBall);

    filter.predictWithUpdate(updateBall.getPos());
}

bool KalmanBall::isUnhealthy() const {
    bool updated_recently =
        RJ::Seconds(lastPredictTime - lastUpdateTime) < RJ::Seconds(PARAM_max_time_outside_vision);

    return !updated_recently;
}

unsigned int KalmanBall::getCameraID() const { return cameraID; }

int KalmanBall::getHealth() const { return health; }

Geometry2d::Point KalmanBall::getPos() const { return filter.getPos(); }

Geometry2d::Point KalmanBall::getVel() const { return filter.getVel(); }

Geometry2d::Point KalmanBall::getPosCov() const { return filter.getPosCov(); }

Geometry2d::Point KalmanBall::getVelCov() const { return filter.getVelCov(); }

const boost::circular_buffer<CameraBall>& KalmanBall::getPrevMeasurements() const {
    return previousMeasurements;
}

void KalmanBall::setVel(Geometry2d::Point newVel) { filter.setVel(newVel); }
}  // namespace vision_filter