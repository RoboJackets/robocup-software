#pragma once

#include <Configuration.hpp>
#include <Geometry2d/Point.hpp>
#include <list>
#include <rj_common/Utils.hpp>

#include "KalmanBall.hpp"

class KalmanBall;

/**
 * Best estimate of the true ball position using the kalman balls from each camera
 */
class WorldBall {
public:
    /**
     * Creates an invalid world ball
     * Used to make some of the code cleaner
     */
    WorldBall();

    /**
     * Creates a valid world ball from a list of kalman balls through special averaging
     *
     * @param calcTime Current iteration time
     * @param kalmanBalls List of best kalman ball from every camera
     */
    WorldBall(RJ::Time calcTime, const std::list<KalmanBall>& kalmanBalls);

    /**
     * @return If the ball actually represents a real ball
     */
    bool getIsValid() const;

    /**
     * @return The best estimated position of the ball
     */
    Geometry2d::Point getPos() const;

    /**
     * @return The best estimated velocity of the ball
     */
    Geometry2d::Point getVel() const;

    /**
     * @return The average position covariance of the filter
     */
    double getPosCov() const;

    /**
     * @return The average velocity covariance of the filter
     */
    double getVelCov() const;

    /**
     * @return List of all the building kalman balls for this world ball
     */
    const std::list<KalmanBall>& getBallComponents() const;

    /**
     * @return Time of creation for this world ball
     */
    RJ::Time getTime() const;

    static void createConfiguration(Configuration* cfg);

private:
    bool isValid;
    Geometry2d::Point pos;
    Geometry2d::Point vel;
    double posCov{};
    double velCov{};
    std::list<KalmanBall> ballComponents;
    RJ::Time time;

    static ConfigDouble* ball_merger_power;
};
