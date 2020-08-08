#pragma once

#include <deque>
#include <rj_vision_filter/ball/world_ball.hpp>
#include <rj_vision_filter/kick/vision_state.hpp>
#include <rj_vision_filter/robot/world_robot.hpp>
#include <utility>
#include <vector>

namespace vision_filter {
/**
 * Contains all the useful information for a kick
 * Like: Who kicked, when, what are the ball positions since etc
 */
class KickEvent {
public:
    /**
     * Creates invalid kick event
     * Makes things a little easier instead of check for null etc
     */
    KickEvent() : is_valid_(false){};

    /**
     * Creates a valid kick event
     *
     * @param kick_time Time of kick
     * @param kicking_robot World robot who is the one kicking
     * @param states_since_kick All the vision states that we have since the kick
     */
    KickEvent(RJ::Time kick_time, WorldRobot kicking_robot,
              std::deque<VisionState> states_since_kick)
        : is_valid_(true),
          kick_time_(kick_time),
          kicking_robot_(std::move(kicking_robot)),
          states_since_kick_(std::move(states_since_kick)) {}

    /**
     * Adds a state to the history
     * Use when the kick event is already created and we are trying
     * to estimate the kick trajectory
     *
     * @param calc_time Time of current frame
     * @param ball Ball at current frame
     * @param yellow_robots Yellow robots at current frame
     * @param blue_robots Blue robots at current frame
     */
    void add_state(RJ::Time calc_time, const WorldBall& ball,
                  const std::vector<WorldRobot>& yellow_robots,
                  const std::vector<WorldRobot>& blue_robots);

    /**
     * @return true if the kick is a valid one
     */
    bool get_is_valid() const;

    /**
     * @return time we think a robot kicked
     */
    RJ::Time get_kick_time() const;

    /**
     * @return robot we think kicked
     */
    WorldRobot get_kicking_robot() const;

    /**
     * @return vision states since that time we kicked
     */
    const std::deque<VisionState>& get_states_since_kick() const;

private:
    // If it's a valid kick event object
    bool is_valid_;
    // When it was kicked
    RJ::Time kick_time_;
    // Who kicked it
    WorldRobot kicking_robot_;
    // All the states since a kick
    std::deque<VisionState> states_since_kick_;
};
}  // namespace vision_filter
