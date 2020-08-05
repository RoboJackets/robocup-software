#include <algorithm>

#include <rj_vision_filter/ball/KalmanBall.hpp>
#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(
    kVisionFilterParamModule, kalman_ball, max_time_outside_vision, 0.2,
    "Max time in seconds that a filter can not be updated before it "
    "is removed.")
using kalman_ball::PARAM_max_time_outside_vision;

KalmanBall::KalmanBall(unsigned int camera_id, RJ::Time creation_time,
                       CameraBall init_measurement,
                       const WorldBall& previous_world_ball)
    : lastUpdateTime(creation_time),
      lastPredictTime(creation_time),
      previousMeasurements(kick::detector::PARAM_slow_kick_hist_length),
      health(filter::health::PARAM_init),
      cameraID(camera_id) {
    Geometry2d::Point init_pos = init_measurement.getPos();
    Geometry2d::Point init_vel = Geometry2d::Point(0, 0);

    // If we have a world ball, use that vel as init to smooth cam transitions
    if (previous_world_ball.getIsValid()) {
        init_vel = previous_world_ball.getVel();
    }

    filter = KalmanFilter2D(init_pos, init_vel);

    previousMeasurements.push_back(init_measurement);
}

void KalmanBall::predict(RJ::Time current_time) {
    lastPredictTime = current_time;

    // Decrement but make sure you don't go too low
    health =
        std::max(health - filter::health::PARAM_dec, filter::health::PARAM_min);

    filter.predict();
}

void KalmanBall::predictAndUpdate(RJ::Time current_time,
                                  CameraBall update_ball) {
    lastPredictTime = current_time;
    lastUpdateTime = current_time;

    // Increment but make sure you don't go too high
    health =
        std::min(health + filter::health::PARAM_inc, filter::health::PARAM_max);

    // Keep last X camera observations in list for kick detection and filtering
    previousMeasurements.push_back(update_ball);

    filter.predictWithUpdate(update_ball.getPos());
}

bool KalmanBall::isUnhealthy() const {
    bool updated_recently = RJ::Seconds(lastPredictTime - lastUpdateTime) <
                            RJ::Seconds(PARAM_max_time_outside_vision);

    return !updated_recently;
}

unsigned int KalmanBall::getCameraID() const { return cameraID; }

int KalmanBall::getHealth() const { return health; }

Geometry2d::Point KalmanBall::getPos() const { return filter.getPos(); }

Geometry2d::Point KalmanBall::getVel() const { return filter.getVel(); }

Geometry2d::Point KalmanBall::getPosCov() const { return filter.getPosCov(); }

Geometry2d::Point KalmanBall::getVelCov() const { return filter.getVelCov(); }

const boost::circular_buffer<CameraBall>& KalmanBall::getPrevMeasurements()
    const {
    return previousMeasurements;
}

void KalmanBall::setVel(Geometry2d::Point new_vel) { filter.setVel(new_vel); }
}  // namespace vision_filter