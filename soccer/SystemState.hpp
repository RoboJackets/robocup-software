#pragma once

#include <vector>
#include <string>
#include <memory>

#include <QMap>
#include <QColor>

#include <Geometry2d/CompositeShape.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Polygon.hpp>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/RadioRx.pb.h>
#include <GameState.hpp>
#include <Constants.hpp>
#include <Utils.hpp>
#include <Geometry2d/Arc.hpp>
#include "planning/MotionInstant.hpp"
#include "planning/Path.hpp"

class RobotConfig;
class OurRobot;
class OpponentRobot;

namespace Packet {
class LogFrame;
}

/**
 * @brief Our beliefs about the ball's position and velocity
 */
class Ball {
public:
    Geometry2d::Point pos;
    Geometry2d::Point vel;
    RJ::Time time;
    bool valid = false;

    Planning::MotionInstant predict(RJ::Time time) const;
    Geometry2d::Point predictPosition(double seconds_from_now) const;

    std::unique_ptr<Planning::Path> path(RJ::Time startTime) const;

    RJ::Time estimateTimeTo(const Geometry2d::Point& point,
                            Geometry2d::Point* nearPoint = nullptr) const;

    double estimateSecondsTo(const Geometry2d::Point &point) const;

    double estimateSecondsToDist(double dist) const;

    double predictSecondsToStop() const;


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

    RJ::Timestamp timestamp() const { return RJ::timestamp(time); }

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

};
