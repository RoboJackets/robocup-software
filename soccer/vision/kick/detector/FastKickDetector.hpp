#include <deque>

#include <Geometry2d/Point.hpp>
#include <Utils.hpp>

#include "vision/ball/WorldBall.hpp"
#include "vision/robot/WorldRobot.hpp"
#include "vision/kick/KickEvent.hpp"
#include "vision/kick/VisionState.hpp"

class FastKickDetector {
public:
    FastKickDetector();

    bool addRecord(RJ::Time calcTime, WorldBall ball,
                   std::vector<WorldRobot> yellowRobots,
                   std::vector<WorldRobot> blueRobots,
                   KickEvent& kickEvent);

private:
    bool detectKick();
    WorldRobot getClosestRobot();

    std::deque<VisionState> stateHistory;
};