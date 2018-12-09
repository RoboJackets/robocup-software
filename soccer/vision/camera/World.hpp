#include <list>
#include <vector>

#include "Utils.hpp"

#include "vision/camera/Camera.hpp"
#include "vision/camera/CameraFrame.hpp"

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"

#include "vision/kick/detector/FastKickDetector.hpp"
#include "vision/kick/detector/SlowKickDetector.hpp"

class World {
public:
    World();

    void updateWithCameraFrame(RJ::Time calcTime, std::list<CameraFrame> newFrames);
    void updateWithoutCameraFrame(RJ::Time calcTime);

    WorldBall getWorldBall();
    std::vector<WorldRobot> getRobotsYellow();
    std::vector<WorldRobot> getRobotsBlue();

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
};