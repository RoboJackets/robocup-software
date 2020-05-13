#pragma once

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <Constants.hpp>
#include <GameState.hpp>
#include <Geometry2d/Arc.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <Utils.hpp>
#include "planning/DynamicObstacle.hpp"
#include "planning/MotionInstant.hpp"
#include "planning/trajectory/Trajectory.hpp"

class RobotConfig;
class OurRobot;
class OpponentRobot;

namespace Packet {
class LogFrame;
}  // namespace Packet

/**
 * @brief Our beliefs about the ball's position and velocity
 */
class Ball {
private:
    Planning::Trajectory _path = Planning::Trajectory{{}};

public:
    Geometry2d::Point pos;
    Geometry2d::Point vel;
    RJ::Time time;
    bool valid = false;

    [[nodiscard]] Planning::MotionInstant predict(RJ::Time estimateTime) const;
    [[nodiscard]] Geometry2d::Point predictPosition(
        double seconds_from_now) const;

    RJ::Time estimateTimeTo(const Geometry2d::Point& point,
                            Geometry2d::Point* nearPointOut = nullptr) const;

    [[nodiscard]] double estimateSecondsTo(
        const Geometry2d::Point& point) const;

    [[nodiscard]] double predictSecondsToStop() const;
    [[nodiscard]] double estimateSecondsToDist(double dist) const;

    Planning::DynamicObstacle dynamicObs();
};

class Context;

/**
 * @brief Holds the positions of everything on the field
 * @details  this has the debugging drawer for the gui
 * but it also contains the game state, so this is passed game state information
 * contains essentially everything data wise
 * used in all threads, this is the class that is passed to for data
 */
class SystemState {
public:
    SystemState(Context* const context);
    ~SystemState();

    /**
     * @defgroup drawing_functions Drawing Functions
     * These drawing functions add certain shapes/lines to the current LogFrame.
     * Each time the FieldView updates, it reads the LogFrame and draws these
     * items.
     * This way debug data can be drawn on-screen and also logged.
     *
     * Each drawing function also associates the drawn content with a particular
     * 'layer'.  Separating drawing items into layers lets you choose at runtime
     * which items actually get drawn.
     */

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

    Ball ball;
    std::shared_ptr<Packet::LogFrame> logFrame;

    std::vector<int> ourValidIds();

    bool paused;
};