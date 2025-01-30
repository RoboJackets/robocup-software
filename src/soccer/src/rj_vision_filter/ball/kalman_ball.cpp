#include <algorithm>

#include <rj_vision_filter/ball/kalman_ball.hpp>
#include <rj_vision_filter/ball/world_ball.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {

DEFINE_NS_FLOAT64(kVisionFilterParamModule, kalman_ball, max_time_outside_vision, 0.2,
                  "Max time in seconds that a filter can not be updated before it "
                  "is removed.")
using kalman_ball::PARAM_max_time_outside_vision;

KalmanBall::KalmanBall(unsigned int camera_id, RJ::Time creation_time, CameraBall init_measurement,
                       const WorldBall& previous_world_ball)
    : last_update_time_(creation_time),
      last_predict_time_(creation_time),
      previous_measurements_(kick::detector::PARAM_slow_kick_hist_length),
      health_(filter::health::PARAM_init),
      camera_id_(camera_id) {
    rj_geometry::Point init_pos = init_measurement.get_pos();
    rj_geometry::Point init_vel = rj_geometry::Point(0, 0);

    // If we have a world ball, use that vel as init to smooth cam transitions
    if (previous_world_ball.get_is_valid()) {
        init_vel = previous_world_ball.get_vel();
    }

    filter_ = KalmanFilter2D(init_pos, init_vel);

    previous_measurements_.push_back(init_measurement);
}

void KalmanBall::predict(RJ::Time current_time) {
    last_predict_time_ = current_time;

    // Decrement but make sure you don't go too low
    health_ = std::max(health_ - filter::health::PARAM_dec, filter::health::PARAM_min);

    filter_.predict();
}

void KalmanBall::predict_and_update(RJ::Time current_time, CameraBall update_ball) {
    last_predict_time_ = current_time;
    last_update_time_ = current_time;

    // Increment but make sure you don't go too high
    health_ = std::min(health_ + filter::health::PARAM_inc, filter::health::PARAM_max);

    // Keep last X camera observations in list for kick detection and filtering
    previous_measurements_.push_back(update_ball);

    filter_.predict_with_update(update_ball.get_pos());
}

bool KalmanBall::is_unhealthy() const {
    bool updated_recently = RJ::Seconds(last_predict_time_ - last_update_time_) <
                            RJ::Seconds(PARAM_max_time_outside_vision);

    return !updated_recently;
}

unsigned int KalmanBall::get_camera_id() const { return camera_id_; }

int KalmanBall::get_health() const { return health_; }

rj_geometry::Point KalmanBall::get_pos() const { return filter_.get_pos(); }

rj_geometry::Point KalmanBall::get_vel() const { return filter_.get_vel(); }

rj_geometry::Point KalmanBall::get_pos_cov() const { return filter_.get_pos_cov(); }

rj_geometry::Point KalmanBall::get_vel_cov() const { return filter_.get_vel_cov(); }

const boost::circular_buffer<CameraBall>& KalmanBall::get_prev_measurements() const {
    return previous_measurements_;
}

void KalmanBall::set_vel(rj_geometry::Point new_vel) { filter_.set_vel(new_vel); }
}  // namespace vision_filter