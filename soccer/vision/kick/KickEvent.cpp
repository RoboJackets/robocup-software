#include "KickEvent.hpp"

void KickEvent::addState(RJ::Time calcTime, WorldBall ball,
                         std::vector<WorldRobot> yellowRobots,
                         std::vector<WorldRobot> blueRobots) {
    statesSinceKick.emplace_back(calcTime, ball, yellowRobots, blueRobots);
}

bool KickEvent::getIsValid() {
    return isValid;
}