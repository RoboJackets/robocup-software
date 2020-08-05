#include <rj_vision_filter/kick/KickEvent.hpp>

namespace vision_filter {
void KickEvent::addState(RJ::Time calc_time, const WorldBall& ball,
                         const std::vector<WorldRobot>& yellow_robots,
                         const std::vector<WorldRobot>& blue_robots) {
    statesSinceKick.emplace_back(calc_time, ball, yellow_robots, blue_robots);
}

bool KickEvent::getIsValid() const { return isValid; }

RJ::Time KickEvent::getKickTime() const { return kickTime; }

WorldRobot KickEvent::getKickingRobot() const { return kickingRobot; }

const std::deque<VisionState>& KickEvent::getStatesSinceKick() const {
    return statesSinceKick;
}
}  // namespace vision_filter