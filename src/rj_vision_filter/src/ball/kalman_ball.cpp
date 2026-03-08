#include <algorithm>

#include <rj_vision_filter/ball/kalman_ball.hpp>
#include <rj_vision_filter/ball/world_ball.hpp>

namespace vision_filter {

KalmanBall::KalmanBall(
    unsigned int camera_id,
    RJ::Time creation_time,
    CameraBall init_measurement,
    const WorldBall& previous_world_ball
) : last_update_time_(creation_time),
    last_predict_time_(creation_time),
    previous_measurements_(3),
    camera_id_(camera_id) {
    rj_geometry::Point init_pos = init_measurement.get_pos();
    rj_geometry::Point init_vel = rj_geometry::Point(0, 0);

    // If we have a world ball, use that vel as init to smooth cam transitions
    if (previous_world_ball.get_is_valid()) {
        init_vel = previous_world_ball.get_vel();
    }

    filter_ = KalmanFilter2D(init_pos, init_vel, vision_loop_dt_, ball_initial_covariance_, ball_process_noise_, ball_observation_noise_);

    previous_measurements_.push_back(init_measurement);
}

KalmanBall::KalmanBall(unsigned int camera_id, RJ::Time creation_time, CameraBall init_measurement,
                       const WorldBall& previous_world_ball, const std::shared_ptr<rclcpp::Node>& vision_filter_node)
    : last_update_time_(creation_time),
      last_predict_time_(creation_time),
      camera_id_(camera_id) {
    initialize_parameters(vision_filter_node);

    rj_geometry::Point init_pos = init_measurement.get_pos();
    rj_geometry::Point init_vel = rj_geometry::Point(0, 0);

    // If we have a world ball, use that vel as init to smooth cam transitions
    if (previous_world_ball.get_is_valid()) {
        init_vel = previous_world_ball.get_vel();
    }

    filter_ = KalmanFilter2D(init_pos, init_vel, vision_loop_dt_, ball_initial_covariance_, ball_process_noise_, ball_observation_noise_);

    previous_measurements_.push_back(init_measurement);

    param_cb_handle_ = vision_filter_node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = update_parameters(params);

        return result;
    });
}

void KalmanBall::predict(RJ::Time current_time) {
    last_predict_time_ = current_time;

    // Decrement but make sure you don't go too low
    health_ = std::max(health_ - health_decrement_, min_health_);

    filter_.predict();
}

void KalmanBall::predict_and_update(RJ::Time current_time, CameraBall update_ball) {
    last_predict_time_ = current_time;
    last_update_time_ = current_time;

    // Increment but make sure you don't go too high
    health_ = std::min(health_ + health_increment_, max_health_);

    // Keep last X camera observations in list for kick detection and filtering
    previous_measurements_.push_back(update_ball);

    filter_.predict_with_update(update_ball.get_pos());
}

bool KalmanBall::is_unhealthy() const {
    bool updated_recently = RJ::Seconds(last_predict_time_ - last_update_time_) <
                            RJ::Seconds(max_time_outside_vision_);

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

void KalmanBall::initialize_parameters(const std::shared_ptr<rclcpp::Node>& vision_filter_node) {
    vision_filter_node->get_parameter<int>("filter.health.max", max_health_);
    vision_filter_node->get_parameter<int>("filter.health.min", min_health_);
    vision_filter_node->get_parameter<int>("filter.health.dec", health_decrement_);
    vision_filter_node->get_parameter<int>("filter.health.inc", health_increment_);
    vision_filter_node->get_parameter<int>("filter.health.init", health_);
    int64_t slow_kick_hist_length = vision_filter_node->get_parameter("kick.detector.slow_kick_hist_lenth").as_int();
    previous_measurements_ = boost::circular_buffer<CameraBall>(slow_kick_hist_length);
    vision_filter_node->get_parameter<double>("kalman_ball.max_time_outside_vision", max_time_outside_vision_);
    vision_filter_node->get_parameter<double>("vision_loop_dt", vision_loop_dt_);
    vision_filter_node->get_parameter<double>("ball.init_covariance", ball_initial_covariance_);
    vision_filter_node->get_parameter<double>("ball.observation_noise", ball_observation_noise_);
    vision_filter_node->get_parameter<double>("ball.process_noise", ball_process_noise_);
}

bool KalmanBall::update_parameters(const std::vector<rclcpp::Parameter>& params) {
    bool result = true;

    for (const auto& param : params) {
        if (param.get_name() == "filter.health.max") {
            max_health_ = static_cast<int>(param.as_int());
            health_ = std::min(health_, max_health_);
        } else if (param.get_name() == "filter.health.min") {
            min_health_ = static_cast<int>(param.as_int());
            health_ = std::max(health_, min_health_);
        } else if (param.get_name() == "filter.health.dec") {
            health_decrement_ = static_cast<int>(param.as_int());
        } else if (param.get_name() == "filter.health.inc") {
            health_increment_ = static_cast<int>(param.as_int());
        } else if (param.get_name() == "kick.detector.slow_kick_hist_length") {
            boost::circular_buffer<CameraBall> new_buffer(param.as_int());
            for (CameraBall ball : previous_measurements_) {
                new_buffer.push_back(ball);
            }
            previous_measurements_ = new_buffer;
        } else if (param.get_name() == "kalman_ball.max_time_outside_vision") {
            max_time_outside_vision_ = param.as_double();
        } else if (param.get_name() == "vision_loop_dt") {
            vision_loop_dt_ = param.as_double();
        } else if (param.get_name() == "ball.init_covariance") {
            ball_initial_covariance_ = param.as_double();
        } else if (param.get_name() == "ball.observation_noise") {
            ball_observation_noise_ = param.as_double();
        } else if (param.get_name() == "ball.process_noise") {
            ball_process_noise_ = param.as_double();
        }
    }

    return result;
}
}  // namespace vision_filter