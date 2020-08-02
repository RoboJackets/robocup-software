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

WorldBall::WorldBall(RJ::Time calcTime,
                     const std::list<KalmanBall>& kalmanBalls)
    : isValid(true), time(calcTime) {
    Geometry2d::Point posAvg = Geometry2d::Point(0, 0);
    Geometry2d::Point velAvg = Geometry2d::Point(0, 0);
    double totalPosWeight = 0;
    double totalVelWeight = 0;

    // Below 1 would invert the ratio of scaling
    // Above 2 would just be super noisy
    if (PARAM_ball_merger_power < 1 || PARAM_ball_merger_power > 2) {
        std::cout << "WARN: ball_merger_power should be between 1 and 2"
                  << std::endl;
    }

    if (kalmanBalls.empty()) {
        std::cout << "ERROR: Zero balls are given to the WorldBall constructor"
                  << std::endl;

        isValid = false;
        pos = posAvg;
        vel = velAvg;
        posCov = 0;
        velCov = 0;

        return;
    }

    for (const KalmanBall& ball : kalmanBalls) {
        // Get the covariance of everything
        // AKA how well we can predict the next measurement
        Geometry2d::Point posCov = ball.getPosCov();
        Geometry2d::Point velCov = ball.getVelCov();

        // Std dev of each state
        // Lower std dev gives better idea of true values
        Geometry2d::Point posStdDev;
        Geometry2d::Point velStdDev;
        posStdDev.x() = std::sqrt(posCov.x());
        posStdDev.y() = std::sqrt(posCov.y());
        velStdDev.x() = std::sqrt(velCov.x());
        velStdDev.y() = std::sqrt(velCov.y());

        // Inversely proportional to how much the filter has been updated
        double filterUncertantity = 1.0 / ball.getHealth();

        // How good of pos/vel estimation in total
        // (This is less efficient than just doing the sqrt(x_cov + y_cov),
        //  but it's a little more clear math-wise)
        double posUncertantity = posStdDev.mag();
        double velUncertantity = velStdDev.mag();

        // Weight better estimates higher
        double filterPosWeight = std::pow(posUncertantity * filterUncertantity,
                                          -PARAM_ball_merger_power);

        double filterVelWeight = std::pow(velUncertantity * filterUncertantity,
                                          -PARAM_ball_merger_power);

        posAvg += filterPosWeight * ball.getPos();
        velAvg += filterVelWeight * ball.getVel();

        totalPosWeight += filterPosWeight;
        totalVelWeight += filterVelWeight;
    }

    posAvg /= totalPosWeight;
    velAvg /= totalVelWeight;

    pos = posAvg;
    vel = velAvg;
    posCov = totalPosWeight / kalmanBalls.size();
    velCov = totalVelWeight / kalmanBalls.size();
    ballComponents = kalmanBalls;
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