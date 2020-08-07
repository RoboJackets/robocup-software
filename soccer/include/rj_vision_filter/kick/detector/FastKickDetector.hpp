#pragma once

#include <deque>
#include <rj_common/Utils.hpp>
#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/kick/KickEvent.hpp>
#include <rj_vision_filter/kick/VisionState.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>

namespace vision_filter {
/**
 * Detects extremely fast kicks in the case where the
 * 5 or more samples required by the slow kick detector
 * would take too long to collect and still allow time to
 * react to the ball.
 *
 * Uses a very simple velocity change over 3 samples to test
 * for kicks.
 */
class FastKickDetector {
public:
    /**
     * Adds a record to our history list
     *
     * @param calc_time Time of calculation for this vision loop
     * @param ball Best estimation of the current ball
     * @param yellow_robots Best estimation of the yellow robots
     * @param blue_robots Best estimation of the blue robots
     * @param kick_event Returned kick event if we find one
     *
     * @return Whether there was a kick
     *
     * @note kick_event is only filled if it returns true
     * It is not touched otherwise
     */
    bool add_record(RJ::Time calc_time, const WorldBall& ball,
                   const std::vector<WorldRobot>& yellow_robots,
                   const std::vector<WorldRobot>& blue_robots,
                   KickEvent& kick_event);

private:
    /**
     * @return Whether there is a large enough acceleration to be a kick
     */
    bool detect_kick();

    /**
     * @return Closest robot to the ball at it's kick time
     */
    WorldRobot get_closest_robot();

    std::deque<VisionState> state_history_;
};
}  // namespace vision_filter
