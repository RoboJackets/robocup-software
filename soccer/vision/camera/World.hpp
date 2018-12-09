#include <list>
#include <vector>

#include <Utils.hpp>
#include <Configuration.hpp>

#include "vision/camera/Camera.hpp"
#include "vision/camera/CameraFrame.hpp"

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"

#include "vision/kick/detector/FastKickDetector.hpp"
#include "vision/kick/detector/SlowKickDetector.hpp"
#include "vision/kick/KickEvent.hpp"

class World {
public:
    World();

    void updateWithCameraFrame(RJ::Time calcTime, std::list<CameraFrame> newFrames);
    void updateWithoutCameraFrame(RJ::Time calcTime);

    WorldBall getWorldBall();
    std::vector<WorldRobot> getRobotsYellow();
    std::vector<WorldRobot> getRobotsBlue();
    KickEvent getBestKickEstimate();

    static void createConfiguration(Configuration* cfg);

private:
    void calcBallBounce();
    void updateWorldObjects(RJ::Time calcTime);
    void detectKicks(RJ::Time calcTime);

    std::vector<Camera> cameras;

    WorldBall ball;
    std::vector<WorldRobot> robotsYellow;
    std::vector<WorldRobot> robotsBlue;

    FastKickDetector fastKick;
    SlowKickDetector slowKick;
    KickEvent bestKickEstimate;

    static ConfigDouble* fast_kick_timeout;
    static ConfigDouble* slow_kick_timeout;
    static ConfigDouble* same_kick_timeout;
};