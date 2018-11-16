#include <vector>

#include <Geometry2d/Point.hpp>
#include <Configuration.hpp>

#include "KalmanBall.hpp"

class WorldBall {
public:
    /**
     * Creates a world ball from a list of kalman balls through special averaging
     *
     * @param kalmanBalls List of best kalman ball from every camera
     */
    WorldBall(std::vector<KalmanBall> kalmanBalls);

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
    std::vector<KalmanBall> getBallComponents();

    static void createConfiguration(Configuration* cfg);

private:
    Geometry2d::Point pos;
    Geometry2d::Point vel;
    double posCov;
    double velCov;
    std::vector<KalmanBall> ballComponents;

    ConfigDouble* ball_merger_power;
};