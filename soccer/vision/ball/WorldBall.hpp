#include <list>

#include <Geometry2d/Point.hpp>
#include <Configuration.hpp>

#include "KalmanBall.hpp"

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
     * @param kalmanBalls List of best kalman ball from every camera
     */
    WorldBall(std::list<KalmanBall> kalmanBalls);

    /**
     * @return If the ball actually represents a real ball
     */
    bool getIsValid();

    /**
     * @return The best estimated position of the ball
     */
    Geometry2d::Point getPos();

    /**
     * @return The best estimated velocity of the ball
     */
    Geometry2d::Point getVel();

    /**
     * @return The average position covariance of the filter
     */
    double getPosCov();

    /**
     * @return The average velocity covariance of the filter
     */
    double getVelCov();

    /**
     * @return List of all the building kalman balls for this world ball
     */
    std::list<KalmanBall> getBallComponents();

    static void createConfiguration(Configuration* cfg);

private:
    bool isValid;
    Geometry2d::Point pos;
    Geometry2d::Point vel;
    double posCov;
    double velCov;
    std::list<KalmanBall> ballComponents;

    ConfigDouble* ball_merger_power;
};