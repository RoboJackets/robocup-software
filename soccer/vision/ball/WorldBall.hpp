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

    Geometry2d::Point getPos();
    Geometry2d::Point getVel();

    /**
     * @return The average position covariance of the filter
     */
    Geometry2d::Point getPosCov();

    /**
     * @return The average velocity covariance of the filter
     */
    Geometry2d::Point getVelCov();

    /**
     * @return List of all the building kalman balls for this world ball
     */
    std::vector<KalmanBall> getBallComponents();

    static void createConfiguration(Configuration* cfg);

private:
    Geometry2d::Point pos;
    Geometry2d::Point vel;
    Geometry2d::Point posCov;
    Geometry2d::Point velCov;
    std::vector<KalmanBall> ballComponents;

    ConfigDouble* ball_merger_power;
};