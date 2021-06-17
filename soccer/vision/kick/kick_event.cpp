#include "vision/kick/kick_event.hpp"

namespace vision_filter {
void KickEvent::add_state(RJ::Time calc_time, const WorldBall& ball,
                          const std::vector<WorldRobot>& yellow_robots,
                          const std::vector<WorldRobot>& blue_robots) {
    states_since_kick_.emplace_back(calc_time, ball, yellow_robots, blue_robots);
}

bool KickEvent::get_is_valid() const { return is_valid_; }

RJ::Time KickEvent::get_kick_time() const { return kick_time_; }

WorldRobot KickEvent::get_kicking_robot() const { return kicking_robot_; }

const std::deque<VisionState>& KickEvent::get_states_since_kick() const {
    return states_since_kick_;
}
}  // namespace vision_filter