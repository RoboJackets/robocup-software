#pragma once

#include <Constants.hpp>
#include <planning/CompositePath.hpp>
#include <planning/InterpolatedPath.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/RobotConstraints.hpp>
#include <planning/RRTPlanner.hpp>
#include "planning/RotationCommand.hpp"
#include "planning/DynamicObstacle.hpp"

#include <protobuf/RadioRx.pb.h>
#include <protobuf/RadioTx.pb.h>
#include <protobuf/Control.pb.h>
#include <Utils.hpp>

#include <array>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <Eigen/Dense>
#include <QColor>
#include <stdint.h>
#include <vector>

#include <QReadWriteLock>
#include <QReadLocker>
#include <QWriteLocker>

#include "firmware-common/robot2015/cpu/status.h"

class SystemState;
class RobotConfig;
class RobotStatus;
class MotionControl;
class RobotFilter;

namespace Packet {
class DebugText;
class LogFrame_Robot;
};

namespace Gameplay {
class GameplayModule;
}

namespace Planning {
class RRTPlanner;
}

/**
 * @brief Contains robot motion state data
 * @details This class contains data that comes from the vision system
 * including position data and which camera this robot was seen by and
 * what time it was last seen.
 */
class RobotPose {
public:
    RobotPose()
        : visible(false),
          velValid(false),
          angle(0),
          angleVel(0),
          time(),
          visionFrame(0) {
        // normalize angle so it's always positive
        // while (angle < 0) angle += 2.0 * M_PI;
    }

    bool visible;
    bool velValid;

    Geometry2d::Point pos;
    Geometry2d::Point vel;
    /// angle in radians.  0 radians means the robot is aimed along the x-axis
    double angle;
    double angleVel;  /// angular velocity in radians/sec

    // Time at which this estimate is valid
    RJ::Time time;
    int visionFrame;
};

class Robot : public RobotPose {
public:
    Robot(unsigned int shell, bool self);
    ~Robot();

    /**
     * ID number for the robot.  This is the number that the dot pattern on the
     * top of the robot represents
     */
    unsigned int shell() const { return _shell; }

    /**
     * Check whether or not this robot is on our team
     */
    bool self() const { return _self; }

    /**
     * Get the robot's position filter.
     */
    RobotFilter* filter() const { return _filter; }

    bool operator==(const Robot& other) {
        return shell() == other.shell() && self() == other.self();
    }

    std::string toString() const {
        return std::string("<Robot ") + (self() ? "us[" : "them[") +
               std::to_string(shell()) + "], pos=" + pos.toString() + ">";
    }

    friend std::ostream& operator<<(std::ostream& stream, const Robot& robot) {
        stream << robot.toString();
        return stream;
    }

private:
    unsigned int _shell;
    bool _self;
    RobotFilter* _filter;
};

/**
 * @brief A robot on our team
 * @details This extends Robot and provides methods for interacting with our
 * robots.
 * A few things this class is responsible for:
 * - specifying target position, velocity, angle, etc
 * - kicking and chipping the ball
 * - keeping track of which hardware revision this bot is
 * - avoidance of the ball and other robots (this info is fed to the path
 * planner)
 * - playing the GT fight song
 */
class OurRobot : public Robot {
public:
    typedef std::array<float, Num_Shells> RobotMask;

    /** radio packets */
    Packet::Robot robotPacket;
    Packet::Control* control;

    RobotConfig* config;
    RobotStatus* status;

    /**
     * @brief Construct a new OurRobot
     * @param shell The robot ID
     * @param state A pointer to the global system state object
     */
    OurRobot(int shell, SystemState* state);
    ~OurRobot();

    void addStatusText();

    void addText(const QString& text, const QColor& color = Qt::white,
                 const QString& layerPrefix = "RobotText");

    /// true if the kicker is ready
    bool charged() const;

    /// returns the time since the kicker was last charged, 0.0 if ready
    float kickTimer() const;

    /// segment for the location of the kicker
    const Geometry2d::Segment kickerBar() const;
    /// converts a point to the frame of reference of robot
    Geometry2d::Point pointInRobotSpace(Geometry2d::Point pt) const;

    // simple checks to do geometry
    /** returns true if the position specified is behind the robot */
    // FIXME - Function name and comment don't match
    bool behindBall(Geometry2d::Point ballPos) const;

    // Gets the robot quaternion.  Returns false (and does not change q) if not
    // available.
    boost::optional<Eigen::Quaternionf> quaternion() const;

    // Constraints
    const RobotConstraints& robotConstraints() const {
        return _robotConstraints;
    }

    RobotConstraints& robotConstraints() { return _robotConstraints; }

    const MotionConstraints& motionConstraints() const {
        return _robotConstraints.mot;
    }

    MotionConstraints& motionConstraints() { return _robotConstraints.mot; }

    const Planning::RotationCommand& rotationCommand() const {
        return *_rotationCommand;
    }

    /**
     * Returns a const reference to the path of the robot.
     */
    const Planning::Path& path() {
        // return *angleFunctionPath.path;
        return angleFunctionPath;
    }

    /// clears old radioTx stuff, resets robot debug text, and clears local
    /// obstacles
    void resetForNextIteration();

    /// clears all fields in the robot's MotionConstraints object, causing the
    /// robot to stop
    void resetMotionConstraints();

    /** Stop the robot */
    void stop();

    /**
     * Makes this robot execute a lineKick
     *
     * @param target - The target to kick towards (aiming point)
     */
    void lineKick(Geometry2d::Point target);

    /**
     * @brief Move to a given point using the default RRT planner
     * @param endSpeed - the speed we should be going when we reach the end of
     * the path
     */
    void move(Geometry2d::Point goal,
              Geometry2d::Point endVelocity = Geometry2d::Point());

    /**
     * @brief Move to a given point bypassing the RRT Path planner. This will
     * plan a direct path ignoring all obstacles and the starting velocity
     * @param endSpeed - the speed we should be going when we reach the end of
     * the path
     */
    void moveDirect(Geometry2d::Point goal, float endSpeed = 0);

    /**
     * @brief Move to a given point while breaking everything. Tells the robot
     * it is already at the endpoint
     * @param endSpeed - the speed we should be going when we reach the end of
     * the path. I'm not even sure if this part makes any sense here.
     */
    void moveTuning(Geometry2d::Point goal, float endSpeed = 0);

    /**
     * Sets the worldVelocity in the robot's MotionConstraints
     */
    void worldVelocity(Geometry2d::Point targetWorldVel);

    /**
     * Face a point while remaining in place
     */
    void face(Geometry2d::Point pt);

    /**
     * Remove the facing command
     */
    void faceNone();

    /**
     * The robot pivots around it's mouth toward the given target
     */
    void pivot(Geometry2d::Point pivotTarget);

    /*
     * Enable dribbler (0 to 127)
     */
    void dribble(uint8_t speed);

    /**
     * KICKING/CHIPPING
     *
     * When we call kick() or chip(), it doesn't happen immediately. It primes
     * the kicker or chipper to kick at the designated power the next time the
     * bot senses that it has the ball.  Once this happens, we record the time
     * of the actual kick.
     */

    /**
     * enable kick when ready at a given percentage of the currently set max
     * kick power.
     * @param strength a value between 0 and 1
     */
    void kick(float strength);

    /**
     * enable kick when ready at a given strength (0-255)
     */
    void kickLevel(uint8_t strength);

    /**
     * enable chip when ready at a given percentage of the currently set max
     * chip power.
     * @param strength a value between 0 and 1
     */
    void chip(float strength);

    /**
     * enable chip when ready at a given strength (0-255)
     */
    void chipLevel(uint8_t strength);

    /**
     * @brief Undoes any calls to kick() or chip().
     */
    void unkick();

    RJ::Timestamp lastKickTime() const;

    /// checks if the bot has kicked/chipped very recently.
    bool justKicked() { return !(_radioRx.kicker_status() & Kicker_Charged); }

    /**
     * Gets a string representing the series of commands called on the robot
     * this iteration. Contains face(), move(), etc - used to display in the
     * BehaviorTree tab in soccer
     */
    std::string getCmdText() const;

    /**
     * ignore ball sense and kick immediately
     */
    void kickImmediately();

    boost::ptr_vector<Packet::DebugText> robotText;

    // True if this robot will treat opponents as obstacles
    // Set to false for defenders to avoid being herded
    bool avoidOpponents() const;
    void avoidOpponents(bool enable);

    void disableAvoidBall();
    void avoidBallRadius(float radius);
    float avoidBallRadius() const;
    void resetAvoidBall();  // sets avoid ball radius to Ball_Avoid_Small

    void resetAvoidRobotRadii();

    /**
     * Adds an obstacle to the local set of obstacles for avoidance
     * Cleared after every frame
     */
    void localObstacles(const std::shared_ptr<Geometry2d::Shape>& obs) {
        _local_obstacles.add(obs);
    }
    const Geometry2d::ShapeSet& localObstacles() const {
        return _local_obstacles;
    }
    void clearLocalObstacles() { _local_obstacles.clear(); }

    std::vector<Planning::DynamicObstacle> collectDynamicObstacles();

    Geometry2d::ShapeSet collectStaticObstacles(
        const Geometry2d::ShapeSet& globalObstacles,
        bool localObstacles = true);

    Geometry2d::ShapeSet collectAllObstacles(
        const Geometry2d::ShapeSet& globalObstacles);

    void approachAllOpponents(bool enable = true);
    void avoidAllOpponents(bool enable = true);

    /** checks if opponents are avoided at all */
    bool avoidOpponent(unsigned shell_id) const;

    /** @return true if we are able to approach the given opponent */
    bool approachOpponent(unsigned shell_id) const;

    /** returns the avoidance radius */
    float avoidOpponentRadius(unsigned shell_id) const;

    /** returns the avoidance radius */
    void avoidAllOpponentRadius(float radius);

    /** enable/disable for opponent avoidance */
    void avoidOpponent(unsigned shell_id, bool enable_avoid);

    /**
     * enable/disable approach of opponents - diable uses larger avoidance
     * radius
     */
    void approachOpponent(unsigned shell_id, bool enable_approach);

    void avoidOpponentRadius(unsigned shell_id, float radius);

    Geometry2d::Point mouthCenterPos() const;

    /**
     * status evaluations for choosing robots in behaviors - combines multiple
     * checks
     */
    bool chipper_available() const;
    bool kicker_available() const;
    bool dribbler_available() const;

    // checks for motor faults - allows one wheel failure if require_all = false
    bool driving_available(bool require_all = true) const;

    // lower level status checks
    bool hasBall() const;
    bool hasBallRaw() const;
    bool ballSenseWorks() const;
    bool kickerWorks() const;
    float kickerVoltage() const;
    Packet::HardwareVersion hardwareVersion() const;

    void setRadioRx(Packet::RadioRx packet) {
        QWriteLocker locker(&radioRxMutex);
        _radioRx = packet;
        if (hasBallRaw()) {
            _lastBallSense = RJ::now();
        }
    }

    Packet::RadioRx radioRx() const {
        QReadLocker locker(&radioRxMutex);
        return _radioRx;
    }

    const std::unique_ptr<Planning::MotionCommand>& motionCommand() const {
        return _motionCommand;
    }

    const RotationConstraints& rotationConstraints() const {
        return _robotConstraints.rot;
    }

    RotationConstraints& rotationConstraints() { return _robotConstraints.rot; }

    MotionControl* motionControl() const { return _motionControl; }

    SystemState* state() const { return _state; }

    /**
     * @param age Time (in microseconds) that defines non-fresh
     */
    bool rxIsFresh(RJ::Seconds age = RJ::Seconds(0.5)) const;

    /**
     * @brief start the robot playing a song
     * @param song
     */
    void sing(Packet::Control::Song song = Packet::Control::FIGHT_SONG) {
        addText("GO TECH!", QColor(255, 0, 255), "Sing");
        control->set_song(song);
    }

    bool isPenaltyKicker = false;
    bool isBallPlacer = false;

    static void createConfiguration(Configuration* cfg);

    double distanceToChipLanding(int chipPower);
    uint8_t chipPowerForDistance(double distance);

    void setPath(std::unique_ptr<Planning::Path> path);

    /**
     * Sets the priority which paths are planned.
     * Higher priority values are planned first.
     */
    void setPlanningPriority(int8_t priority) { _planningPriority = priority; }

    /**
     * Gets the priority which paths are planned.
     * Higher priority values are planned first.
     */
    int8_t getPlanningPriority() { return _planningPriority; }

    void setPID(double p, double i, double d);

protected:
    MotionControl* _motionControl;

    SystemState* _state;

    /// set of obstacles added by plays
    Geometry2d::ShapeSet _local_obstacles;

    /// masks for obstacle avoidance
    RobotMask _opp_avoid_mask;
    float _avoidBallRadius;  /// radius of ball obstacle

    std::unique_ptr<Planning::MotionCommand> _motionCommand;
    std::unique_ptr<Planning::RotationCommand> _rotationCommand;
    RobotConstraints _robotConstraints;

    Planning::AngleFunctionPath angleFunctionPath;  /// latest path

    /**
     * Creates a set of obstacles from a given robot team mask,
     * where mask values < 0 create no obstacle, and larger values
     * create an obstacle of a given radius
     *
     * NOTE: mask must not be set for this robot
     *
     * @param robots is the set of robots to use to create a mask - either self
     * or opp from _state
     */
    template <class ROBOT>
    Geometry2d::ShapeSet createRobotObstacles(const std::vector<ROBOT*>& robots,
                                              const RobotMask& mask) const {
        Geometry2d::ShapeSet result;
        for (size_t i = 0; i < mask.size(); ++i)
            if (mask[i] > 0 && robots[i] && robots[i]->visible)
                result.add(std::shared_ptr<Geometry2d::Shape>(
                    new Geometry2d::Circle(robots[i]->pos, mask[i])));
        return result;
    }

    /**
     * Only adds obstacles within the checkRadius of the passed in position
     * Creates a set of obstacles from a given robot team mask, where mask
     * values < 0 create no obstacle, and larger values create an obstacle of a
     * given radius
     *
     * NOTE: mask must not be set for this robot
     *
     * @param robots is the set of robots to use to create a mask - either self
     * or opp from _state
     */
    template <class ROBOT>
    Geometry2d::ShapeSet createRobotObstacles(const std::vector<ROBOT*>& robots,
                                              const RobotMask& mask,
                                              Geometry2d::Point currentPosition,
                                              float checkRadius) const {
        Geometry2d::ShapeSet result;
        for (size_t i = 0; i < mask.size(); ++i)
            if (mask[i] > 0 && robots[i] && robots[i]->visible) {
                if (currentPosition.distTo(robots[i]->pos) <= checkRadius) {
                    result.add(std::shared_ptr<Geometry2d::Shape>(
                        new Geometry2d::Circle(robots[i]->pos, mask[i])));
                }
            }
        return result;
    }

    /**
     * Creates an obstacle for the ball if necessary
     */
    std::shared_ptr<Geometry2d::Circle> createBallObstacle() const;

    friend class Processor;

    /// The processor mutates RadioRx in place and calls this afterwards to let
    /// it know that it changed
    void radioRxUpdated();

    friend class MotionControl;

private:
    RJ::Time _lastBallSense;
    const RJ::Seconds _lostBallDuration = RJ::Seconds(0.1);

    mutable QReadWriteLock radioRxMutex;
    void _kick(uint8_t strength);
    void _chip(uint8_t strength);
    void _unkick();

    uint32_t _lastKickerStatus;
    RJ::Time _lastKickTime;
    RJ::Time _lastChargedTime;

    Packet::RadioRx _radioRx;

    /**
     * We build a string of commands such as face(), move(), etc at each
     * iteration
     * Then display this in the BehaviorTree tab in soccer
     */
    // note: originally this was not a pointer, but I got weird errors about a
    // deleted copy constructor...
    std::stringstream* _cmdText;

    void _clearCmdText();

    /// default values for avoid radii
    static ConfigDouble* _selfAvoidRadius;
    static ConfigDouble* _oppAvoidRadius;
    static ConfigDouble* _oppGoalieAvoidRadius;

    int8_t _planningPriority;
};

/**
 * @brief A robot that is not on our team
 * @details This is a subclass of Robot, but really doesn't provide
 * any extra functionality.
 */
class OpponentRobot : public Robot {
public:
    OpponentRobot(unsigned int shell) : Robot(shell, false) {}
};
