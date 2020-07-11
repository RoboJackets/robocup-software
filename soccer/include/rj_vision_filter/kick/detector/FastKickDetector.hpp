#pragma once

#include <Configuration.hpp>
#include <deque>
#include <rj_common/Utils.hpp>

#include "vision/ball/WorldBall.hpp"
#include "vision/kick/KickEvent.hpp"
#include "vision/kick/VisionState.hpp"
#include "vision/robot/WorldRobot.hpp"

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
     * @param calcTime Time of calculation for this vision loop
     * @param ball Best estimation of the current ball
     * @param yellowRobots Best estimation of the yellow robots
     * @param blueRobots Best estimation of the blue robots
     * @param kickEvent Returned kick event if we find one
     *
     * @return Whether there was a kick
     *
     * @note kickEvent is only filled if it returns true
     * It is not touched otherwise
     */
    bool addRecord(RJ::Time calcTime, const WorldBall& ball,
                   const std::vector<WorldRobot>& yellowRobots,
                   const std::vector<WorldRobot>& blueRobots,
                   KickEvent& kickEvent);

    static void createConfiguration(Configuration* cfg);

private:
    /**
     * @return Whether there is a large enough acceleration to be a kick
     */
    bool detectKick();

    /**
     * @return Closest robot to the ball at it's kick time
     */
    WorldRobot getClosestRobot();

    std::deque<VisionState> stateHistory;

    // How large of acceleration needed to trigger this detector
    static ConfigDouble* acceleration_trigger;
};
