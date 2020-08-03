#pragma once

#include <deque>
#include <rj_common/Utils.hpp>
#include <rj_vision_filter/ball/WorldBall.hpp>
#include <rj_vision_filter/kick/VisionState.hpp>
#include <rj_vision_filter/robot/WorldRobot.hpp>
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
    KickEvent() : isValid(false){};

    /**
     * Creates a valid kick event
     *
     * @param kickTime Time of kick
     * @param kickingRobot World robot who is the one kicking
     * @param statesSinceKick All the vision states that we have since the kick
     */
    KickEvent(RJ::Time kickTime, WorldRobot kickingRobot,
              std::deque<VisionState> statesSinceKick)
        : isValid(true),
          kickTime(kickTime),
          kickingRobot(std::move(kickingRobot)),
          statesSinceKick(std::move(statesSinceKick)) {}

    /**
     * Adds a state to the history
     * Use when the kick event is already created and we are trying
     * to estimate the kick trajectory
     *
     * @param calcTime Time of current frame
     * @param ball Ball at current frame
     * @param yellowRobots Yellow robots at current frame
     * @param blueRobots Blue robots at current frame
     */
    void addState(RJ::Time calcTime, const WorldBall& ball,
                  const std::vector<WorldRobot>& yellowRobots,
                  const std::vector<WorldRobot>& blueRobots);

    /**
     * @return true if the kick is a valid one
     */
    bool getIsValid() const;

    /**
     * @return time we think a robot kicked
     */
    RJ::Time getKickTime() const;

    /**
     * @return robot we think kicked
     */
    WorldRobot getKickingRobot() const;

    /**
     * @return vision states since that time we kicked
     */
    const std::deque<VisionState>& getStatesSinceKick() const;

private:
    // If it's a valid kick event object
    bool isValid;
    // When it was kicked
    RJ::Time kickTime;
    // Who kicked it
    WorldRobot kickingRobot;
    // All the states since a kick
    std::deque<VisionState> statesSinceKick;
};
}  // namespace vision_filter
