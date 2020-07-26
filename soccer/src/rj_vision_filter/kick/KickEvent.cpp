#include <rj_vision_filter/kick/KickEvent.hpp>

void KickEvent::addState(RJ::Time calcTime, const WorldBall& ball,
                         const std::vector<WorldRobot>& yellowRobots,
                         const std::vector<WorldRobot>& blueRobots) {
    statesSinceKick.emplace_back(calcTime, ball, yellowRobots, blueRobots);
}

bool KickEvent::getIsValid() const {
    return isValid;
}

RJ::Time KickEvent::getKickTime() const {
    return kickTime;
}

WorldRobot KickEvent::getKickingRobot() const {
    return kickingRobot;
}

const std::deque<VisionState>& KickEvent::getStatesSinceKick() const {
    return statesSinceKick;
}