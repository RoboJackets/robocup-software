#pragma once

#include <game_state.hpp>
#include <rj_constants/constants.hpp>
#include <vector>

class OurRobot;
class OpponentRobot;
class BallState;

namespace Packet {
class LogFrame;
}  // namespace Packet

class Context;

/**
 * @brief Holds helper objects for everything on the field. This is being phased
 * out on the C++ side, but is still in use throughout the python side (which
 * needs to access C++ helpers i.e. in the OurRobot class). This will be removed
 * with the move to ROS.
 */
class SystemState {
public:
    SystemState(Context* const context);
    ~SystemState();

    RJ::Time time;

    [[nodiscard]] RJ::Timestamp timestamp() const {
        return RJ::timestamp(time);
    }

    /// All possible robots.
    ///
    /// Robots that aren't on the field are present here because a robot may be
    /// removed and replaced, and that particular robot may be important (e.g.
    /// goalie).
    ///
    /// Plays need to keep Robot*'s around, so we can't just delete the robot
    /// since the play needs to see that it is no longer visible.  We don't want
    /// multiple Robots for the same shell because that would give the
    /// appearance that a new robot appeared when it was actually just pushed
    /// back on the field.
    std::vector<OurRobot*> self;
    std::vector<OpponentRobot*> opp;

    /**
     * A reference to the ball state. This is a helper for python code.
     */
    BallState* ball;

    std::vector<RobotId> our_valid_ids();

    bool paused;
};