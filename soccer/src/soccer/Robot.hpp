#pragma once

#include <rj_common/status.h>

#include <QColor>
#include <algorithm>
#include <array>
#include <boost/circular_buffer.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <cstdint>
#include <optional>
#include <planning/RobotConstraints.hpp>
#include <planning/Trajectory.hpp>
#include <planning/planner/MotionCommand.hpp>
#include <rj_constants/constants.hpp>
#include <vector>

#include "Context.hpp"

class RobotConfig;
class RobotLocalConfig;

namespace Gameplay {
class GameplayModule;
}  // namespace Gameplay

class Robot {
public:
    Robot(Context* context, int shell, bool self);

    /**
     * Get an immutable reference to the robot's estimated state from vision.
     * @return An immutable reference to the robot's state.
     */
    [[nodiscard]] const RobotState& state() const {
        return _context->world_state.get_robot(self(), shell());
    }

    /**
     * Mutable state accessor. Should only be used by vision and tests that
     * are supposed to bypass vision functionality.
     * @return A mutable reference to the robot's state.
     */
    RobotState& mutable_state() {
        return _context->world_state.get_robot(self(), shell());
    }

    [[nodiscard]] Geometry2d::Pose pose() const { return state().pose; }

    [[nodiscard]] Geometry2d::Point pos() const {
        return state().pose.position();
    }

    [[nodiscard]] double angle() const { return state().pose.heading(); }

    [[nodiscard]] Geometry2d::Twist twist() const { return state().velocity; }

    [[nodiscard]] Geometry2d::Point vel() const {
        return state().velocity.linear();
    }

    [[nodiscard]] double angleVel() const { return state().velocity.angular(); }

    [[nodiscard]] bool visible() const { return state().visible; }

    /**
     * ID number for the robot.  This is the number that the dot pattern on
     * the top of the robot represents
     */
    [[nodiscard]] int shell() const { return _shell; }

    /**
     * Check whether or not this robot is on our team
     */
    [[nodiscard]] bool self() const { return _self; }

    bool operator==(const Robot& other) const {
        return shell() == other.shell() && self() == other.self();
    }

    [[nodiscard]] std::string toString() const {
        return std::string("<Robot ") + (self() ? "us[" : "them[") +
               std::to_string(shell()) + "], pos=" + pos().toString() + ">";
    }

    friend std::ostream& operator<<(std::ostream& stream, const Robot& robot) {
        stream << robot.toString();
        return stream;
    }

protected:
    Context* _context;

private:
    const int _shell;
    const bool _self;
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
    using RobotMask = std::array<float, Num_Shells>;

    /**
     * @brief Construct a new OurRobot
     * @param context A pointer to the global system context object
     * @param shell The robot ID
     */
    OurRobot(Context* context, int shell);

    void addStatusText();

    void addText(const QString& text, const QColor& qc = Qt::white,
                 const QString& layerPrefix = "RobotText");

    /// true if the kicker is ready
    bool charged() const;

    /// returns the time since the kicker was last charged, 0.0 if ready
    float kickTimer() const;

    /// segment for the location of the kicker
    Geometry2d::Segment kickerBar() const;
    /// converts a point to the frame of reference of robot
    Geometry2d::Point pointInRobotSpace(Geometry2d::Point pt) const;

    // simple checks to do geometry
    /** returns true if the position specified is behind the robot */
    // FIXME - Function name and comment don't match
    bool behindBall(Geometry2d::Point ballPos) const;

    // Constraints
    const RobotConstraints& robotConstraints() const {
        return _context->robot_constraints[shell()];
    }

    RobotConstraints& robotConstraints() {
        return _context->robot_constraints[shell()];
    }

    const MotionConstraints& motionConstraints() const {
        return robotConstraints().mot;
    }

    MotionConstraints& motionConstraints() { return robotConstraints().mot; }

    /**
     * Returns a const reference to the path of the robot.
     */
    const Planning::Trajectory& path() const {
        return _context->trajectories[shell()];
    }

    /**
     * Returns a movable reference to the path of the robot.
     */
    Planning::Trajectory&& path_movable() {
        return std::move(_context->trajectories[shell()]);
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
     * Intercept the ball as quickly as possible
     * May just slam into the ball if it does not have time to stop
     *
     * @param target - The target position to intercept the ball at
     */
    void intercept(Geometry2d::Point target);

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
     * @brief Move in front of the ball to intercept it. If a target face point
     * is given, the robot will try to face in that direction when the ball
     * hits.
     */
    void settle(std::optional<Geometry2d::Point> target);

    /**
     * @brief Approaches the ball and moves through it slowly
     */
    void collect();

    /*
     * Override the default angle planning strategy and face a point
     */
    void face(Geometry2d::Point pt);

    /**
     * Sets the worldVelocity in the robot's MotionConstraints
     */
    void worldVelocity(Geometry2d::Point targetWorldVel);

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

    const RobotStatus& radioStatus() const {
        return _context->robot_status.at(shell());
    }

    /// checks if the bot has kicked/chipped very recently.
    bool justKicked() {
        return radioStatus().kicker == RobotStatus::KickerState::kCharging;
    }

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
        intent().local_obstacles.add(obs);
    }
    const Geometry2d::ShapeSet& localObstacles() const {
        return intent().local_obstacles;
    }
    void clearLocalObstacles() { intent().local_obstacles.clear(); }

    Geometry2d::ShapeSet collectStaticObstacles(
        const Geometry2d::ShapeSet& globalObstacles,
        bool localObstacles = true);

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
    double kickerVoltage() const;
    RobotStatus::HardwareVersion hardwareVersion() const;

    const Planning::MotionCommand& motionCommand() const {
        return intent().motion_command;
    }
    void setMotionCommand(const Planning::MotionCommand& newCmd) {
        if (intent().motion_command.index() != newCmd.index()) {
            // clear path when command type changes
            _context->trajectories[shell()] = Planning::Trajectory{{}};
        }

        intent().motion_command = newCmd;
    }

    const RotationConstraints& rotationConstraints() const {
        return robotConstraints().rot;
    }

    RotationConstraints& rotationConstraints() {
        return robotConstraints().rot;
    }

    /**
     * @param age Time (in microseconds) that defines non-fresh
     */
    bool statusIsFresh(RJ::Seconds age = RJ::Seconds(0.5)) const;

    /**
     * @brief start the robot playing a song
     * @param song
     */
    void sing(RobotIntent::Song song = RobotIntent::Song::FIGHT_SONG) {
        addText("GO TECH!", QColor(255, 0, 255), "Sing");
        intent().song = song;
    }

    bool isPenaltyKicker = false;
    bool isBallPlacer = false;

    static void createConfiguration(Configuration* cfg);

    double distanceToChipLanding(int chipPower);
    uint8_t chipPowerForDistance(double distance);

    /**
     * Sets the priority which paths are planned.
     * Higher priority values are planned first.
     */
    void setPlanningPriority(int8_t priority) { _planningPriority = priority; }

    /**
     * Gets the priority which paths are planned.
     * Higher priority values are planned first.
     */
    int8_t getPlanningPriority() const { return _planningPriority; }

    void setPID(double p, double i, double d);

    void setJoystickControlled(bool joystickControlled);
    bool isJoystickControlled() const;

protected:
    RobotConstraints _robotConstraints;

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
        for (size_t i = 0; i < mask.size(); ++i) {
            if (mask[i] > 0 && robots[i] && robots[i]->visible()) {
                result.add(std::make_shared<Geometry2d::Circle>(
                    robots[i]->pos(), mask[i]));
            }
        }
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
        for (size_t i = 0; i < mask.size(); ++i) {
            if (mask[i] > 0 && robots[i] && robots[i]->visible()) {
                if (currentPosition.distTo(robots[i]->pos()) <= checkRadius) {
                    result.add(std::make_shared<Geometry2d::Circle>(
                        robots[i]->pos(), mask[i]));
                }
            }
        }
        return result;
    }

    /**
     * Creates an obstacle for the ball if necessary
     */
    std::shared_ptr<Geometry2d::Circle> createBallObstacle() const;

    friend class Processor;
    friend class RadioNode;

    /// The processor mutates RadioRx in place and calls this afterwards to let
    /// it know that it changed
    void radioRxUpdated();

    const RobotLocalConfig* status() const {
        return &_context->local_configs[shell()];
    }

    const RobotConfig* config() const { return _context->robot_config.get(); }

private:
    RJ::Time _lastBallSense;
    const RJ::Seconds _lostBallDuration = RJ::Seconds(0.1);

    void _kick(uint8_t strength);
    void _chip(uint8_t strength);
    void _unkick();

    RobotStatus::KickerState _lastKickerStatus =
        RobotStatus::KickerState::kCharging;
    RJ::Time _lastKickTime;
    RJ::Time _lastChargedTime;

    RobotIntent& intent() { return _context->robot_intents[shell()]; }
    const RobotIntent& intent() const {
        return _context->robot_intents[shell()];
    }

    /**
     * We build a string of commands such as face(), move(), etc at each
     * iteration
     * Then display this in the BehaviorTree tab in soccer
     */
    // note: originally this was not a pointer, but I got weird errors about a
    // deleted copy constructor...
    std::stringstream _cmdText;

    void _clearCmdText();

    /// default values for avoid radii
    static ConfigDouble* _selfAvoidRadius;
    static ConfigDouble* _oppAvoidRadius;
    static ConfigDouble* _oppGoalieAvoidRadius;
    static ConfigDouble* _dribbleOutOfBoundsOffset;

    int8_t _planningPriority{};
};

/**
 * @brief A robot that is not on our team
 * @details This is a subclass of Robot, but really doesn't provide
 * any extra functionality.
 */
class OpponentRobot : public Robot {
public:
    /**
     * @brief Construct a new OpponentRobot
     * @param context A pointer to the global system context object
     * @param shell The robot ID
     */
    OpponentRobot(Context* context, int shell) : Robot(context, shell, false) {}
};
