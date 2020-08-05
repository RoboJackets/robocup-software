#include <cmath>
#include <iostream>

#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/params.hpp>

namespace vision_filter {
DEFINE_NS_FLOAT64(kVisionFilterParamModule, world_ball, ball_merger_power, 1.5,
                  "Multiplier to scale the weighted average coefficient"
                  "to be nonlinear.")
using world_ball::PARAM_ball_merger_power;

WorldBall::WorldBall() : isValid(false) {}

WorldBall::WorldBall(RJ::Time calc_time,
                     const std::list<KalmanBall>& kalman_balls)
    : isValid(true), time(calc_time) {
    Geometry2d::Point pos_avg = Geometry2d::Point(0, 0);
    Geometry2d::Point vel_avg = Geometry2d::Point(0, 0);
    double total_pos_weight = 0;
    double total_vel_weight = 0;

    // Below 1 would invert the ratio of scaling
    // Above 2 would just be super noisy
    if (PARAM_ball_merger_power < 1 || PARAM_ball_merger_power > 2) {
        std::cout << "WARN: ball_merger_power should be between 1 and 2"
                  << std::endl;
    }

    if (kalman_balls.empty()) {
        throw std::runtime_error(
            "ERROR: Zero balls are given to the WorldBall constructor");
    }

    for (const KalmanBall& ball : kalman_balls) {
        // Get the covariance of everything
        // AKA how well we can predict the next measurement
        Geometry2d::Point pos_cov = ball.getPosCov();
        Geometry2d::Point vel_cov = ball.getVelCov();

        // Std dev of each state
        // Lower std dev gives better idea of true values
        Geometry2d::Point pos_std_dev;
        Geometry2d::Point vel_std_dev;
        pos_std_dev.x() = std::sqrt(pos_cov.x());
        pos_std_dev.y() = std::sqrt(pos_cov.y());
        vel_std_dev.x() = std::sqrt(vel_cov.x());
        vel_std_dev.y() = std::sqrt(vel_cov.y());

        // Inversely proportional to how much the filter has been updated
        double filter_uncertantity = 1.0 / ball.getHealth();

        // How good of pos/vel estimation in total
        // (This is less efficient than just doing the sqrt(x_cov + y_cov),
        //  but it's a little more clear math-wise)
        double pos_uncertantity = pos_std_dev.mag();
        double vel_uncertantity = vel_std_dev.mag();

        // Weight better estimates higher
        double filter_pos_weight = std::pow(
            pos_uncertantity * filter_uncertantity, -PARAM_ball_merger_power);

        double filter_vel_weight = std::pow(
            vel_uncertantity * filter_uncertantity, -PARAM_ball_merger_power);

        pos_avg += filter_pos_weight * ball.getPos();
        vel_avg += filter_vel_weight * ball.getVel();

        total_pos_weight += filter_pos_weight;
        total_vel_weight += filter_vel_weight;
    }

    pos_avg /= total_pos_weight;
    vel_avg /= total_vel_weight;

    pos = pos_avg;
    vel = vel_avg;
    posCov = total_pos_weight / kalman_balls.size();
    velCov = total_vel_weight / kalman_balls.size();
    ballComponents = kalman_balls;
}

bool WorldBall::getIsValid() const { return isValid; }

Geometry2d::Point WorldBall::getPos() const { return pos; }

Geometry2d::Point WorldBall::getVel() const { return vel; }

double WorldBall::getPosCov() const { return posCov; }

double WorldBall::getVelCov() const { return velCov; }

const std::list<KalmanBall>& WorldBall::getBallComponents() const {
    return ballComponents;
}

RJ::Time WorldBall::getTime() const { return time; }
}  // namespace vision_filter