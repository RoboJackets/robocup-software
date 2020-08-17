#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_vision_filter/ball/world_ball.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(kVisionFilterParamModule, world_ball, ball_merger_power, 1.5,
                  "Multiplier to scale the weighted average coefficient"
                  "to be nonlinear.")
using world_ball::PARAM_ball_merger_power;

WorldBall::WorldBall() : is_valid_(false) {}

WorldBall::WorldBall(RJ::Time calc_time, const std::list<KalmanBall>& kalman_balls)
    : is_valid_(true), time_(calc_time) {
    rj_geometry::Point pos_avg = rj_geometry::Point(0, 0);
    rj_geometry::Point vel_avg = rj_geometry::Point(0, 0);
    double total_pos_weight = 0;
    double total_vel_weight = 0;

    // Below 1 would invert the ratio of scaling
    // Above 2 would just be super noisy
    if (PARAM_ball_merger_power < 1 || PARAM_ball_merger_power > 2) {
        SPDLOG_WARN("ball_merger_power should be between 1 and 2");
    }

    if (kalman_balls.empty()) {
        spdlog::critical("Zero balls are given to the WorldBall constructor");
    }

    for (const KalmanBall& ball : kalman_balls) {
        // Get the covariance of everything
        // AKA how well we can predict the next measurement
        rj_geometry::Point pos_cov = ball.get_pos_cov();
        rj_geometry::Point vel_cov = ball.get_vel_cov();

        // Std dev of each state
        // Lower std dev gives better idea of true values
        rj_geometry::Point pos_std_dev;
        rj_geometry::Point vel_std_dev;
        pos_std_dev.x() = std::sqrt(pos_cov.x());
        pos_std_dev.y() = std::sqrt(pos_cov.y());
        vel_std_dev.x() = std::sqrt(vel_cov.x());
        vel_std_dev.y() = std::sqrt(vel_cov.y());

        // Inversely proportional to how much the filter has been updated
        double filter_uncertantity = 1.0 / ball.get_health();

        // How good of pos/vel estimation in total
        // (This is less efficient than just doing the sqrt(x_cov + y_cov),
        //  but it's a little more clear math-wise)
        double pos_uncertantity = pos_std_dev.mag();
        double vel_uncertantity = vel_std_dev.mag();

        // Weight better estimates higher
        double filter_pos_weight =
            std::pow(pos_uncertantity * filter_uncertantity, -PARAM_ball_merger_power);

        double filter_vel_weight =
            std::pow(vel_uncertantity * filter_uncertantity, -PARAM_ball_merger_power);

        pos_avg += filter_pos_weight * ball.get_pos();
        vel_avg += filter_vel_weight * ball.get_vel();

        total_pos_weight += filter_pos_weight;
        total_vel_weight += filter_vel_weight;
    }

    pos_avg /= total_pos_weight;
    vel_avg /= total_vel_weight;

    pos_ = pos_avg;
    vel_ = vel_avg;
    pos_cov_ = total_pos_weight / kalman_balls.size();
    vel_cov_ = total_vel_weight / kalman_balls.size();
    ball_components_ = kalman_balls;
}

bool WorldBall::get_is_valid() const { return is_valid_; }

rj_geometry::Point WorldBall::get_pos() const { return pos_; }

rj_geometry::Point WorldBall::get_vel() const { return vel_; }

double WorldBall::get_pos_cov() const { return pos_cov_; }

double WorldBall::get_vel_cov() const { return vel_cov_; }

const std::list<KalmanBall>& WorldBall::get_ball_components() const { return ball_components_; }

RJ::Time WorldBall::get_time() const { return time_; }
}  // namespace vision_filter