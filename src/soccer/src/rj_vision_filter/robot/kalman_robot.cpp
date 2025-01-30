#include <rj_vision_filter/robot/kalman_robot.hpp>

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>

#include <rj_vision_filter/params.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(kVisionFilterParamModule, kalman_robot, max_time_outside_vision, 0.5,
                  "Max number of seconds without a measurement before the object is deleted")
using kalman_robot::PARAM_max_time_outside_vision;

KalmanRobot::KalmanRobot(unsigned int camera_id, RJ::Time creation_time,
                         CameraRobot init_measurement, const WorldRobot& previous_world_robot)
    : last_update_time_(creation_time),
      last_predict_time_(creation_time),
      previous_measurements_(kick::detector::PARAM_slow_kick_hist_length),
      unwrap_theta_ctr_(0),
      health_(filter::health::PARAM_init),
      robot_id_(init_measurement.get_robot_id()),
      camera_id_(camera_id) {
    rj_geometry::Pose init_pose = init_measurement.get_pose();
    rj_geometry::Twist init_twist(0, 0, 0);

    if (previous_world_robot.get_is_valid()) {
        init_twist.linear() = previous_world_robot.get_vel();
        init_twist.angular() = previous_world_robot.get_omega();
    }

    filter_ = KalmanFilter3D(init_pose, init_twist);

    previous_measurements_.push_back(init_measurement);
    previous_theta_ = init_twist.angular();
}

void KalmanRobot::predict(RJ::Time current_time) {
    last_predict_time_ = current_time;

    // Decrement but make sure you don't go too low
    health_ = std::max(health_ - filter::health::PARAM_dec, filter::health::PARAM_min);

    filter_.predict();
}

void KalmanRobot::predict_and_update(RJ::Time current_time, CameraRobot update_robot) {
    last_predict_time_ = current_time;
    last_update_time_ = current_time;

    // Increment but make sure you don't go too high
    health_ = std::min(health_ + filter::health::PARAM_inc, filter::health::PARAM_max);

    // Keep last X camera observations in list for kick detection and filtering
    previous_measurements_.push_back(update_robot);

    // Unwrap theta so we have a continuous heading
    double cur_theta = update_robot.get_theta();

    // See if it went below -pi
    // Note: PI/2 is used to give a good buffer on either side
    if (previous_theta_ < -M_PI_2 && cur_theta > M_PI_2) {
        unwrap_theta_ctr_--;
        // Went above pi
    } else if (previous_theta_ > M_PI_2 && cur_theta < -M_PI_2) {
        unwrap_theta_ctr_++;
    }

    previous_theta_ = cur_theta;

    filter_.predict_with_update({update_robot.get_pos(), cur_theta + unwrap_theta_ctr_ * 2 * M_PI});
}

bool KalmanRobot::is_unhealthy() const {
    bool updated_recently = RJ::Seconds(last_predict_time_ - last_update_time_) <
                            RJ::Seconds(PARAM_max_time_outside_vision);

    return !updated_recently;
}

unsigned int KalmanRobot::get_camera_id() const { return camera_id_; }

int KalmanRobot::get_robot_id() const { return robot_id_; }

int KalmanRobot::get_health() const { return health_; }

rj_geometry::Point KalmanRobot::get_pos() const { return filter_.get_pos(); }

double KalmanRobot::get_theta() const { return filter_.get_theta(); }

rj_geometry::Point KalmanRobot::get_vel() const { return filter_.get_vel(); }

double KalmanRobot::get_omega() const { return filter_.get_omega(); }

rj_geometry::Point KalmanRobot::get_pos_cov() const { return filter_.get_pos_cov(); }

double KalmanRobot::get_theta_cov() const { return filter_.get_theta_cov(); }

rj_geometry::Point KalmanRobot::get_vel_cov() const { return filter_.get_vel_cov(); }

double KalmanRobot::get_omega_cov() const { return filter_.get_omega_cov(); }

const boost::circular_buffer<CameraRobot>& KalmanRobot::get_prev_measurements() const {
    return previous_measurements_;
}
}  // namespace vision_filter