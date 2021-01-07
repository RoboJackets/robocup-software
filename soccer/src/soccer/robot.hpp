#pragma once

#include <rj_common/status.hpp>

#include <QColor>
#include <algorithm>
#include <array>
#include <boost/circular_buffer.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <cstdint>
#include <optional>
#include <planning/robot_constraints.hpp>
#include <planning/trajectory.hpp>
#include <planning/planner/motion_command.hpp>
#include <rj_constants/constants.hpp>
#include <vector>

#include "context.hpp"

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
        return context_->world_state.get_robot(self(), shell());
    }

    /**
     * Mutable state accessor. Should only be used by vision and tests that
     * are supposed to bypass vision functionality.
     * @return A mutable reference to the robot's state.
     */
    RobotState& mutable_state() {
        return context_->world_state.get_robot(self(), shell());
    }

    [[nodiscard]] rj_geometry::Pose pose() const { return state().pose; }

    [[nodiscard]] rj_geometry::Point pos() const {
        return state().pose.position();
    }

    [[nodiscard]] double angle() const { return state().pose.heading(); }

    [[nodiscard]] rj_geometry::Twist twist() const { return state().velocity; }

    [[nodiscard]] rj_geometry::Point vel() const {
        return state().velocity.linear();
    }

    [[nodiscard]] double angle_vel() const { return state().velocity.angular(); }

    [[nodiscard]] bool visible() const { return state().visible; }

    /**
     * ID number for the robot.  This is the number that the dot pattern on
     * the top of the robot represents
     */
    [[nodiscard]] int shell() const { return shell_; }

    /**
     * Check whether or not this robot is on our team
     */
    [[nodiscard]] bool self() const { return self_; }

    bool operator==(const Robot& other) const {
        return shell() == other.shell() && self() == other.self();
    }

    [[nodiscard]] std::string to_string() const {
        return std::string("<Robot ") + (self() ? "us[" : "them[") +
               std::to_string(shell()) + "], pos=" + pos().to_string() + ">";
    }

    friend std::ostream& operator<<(std::ostream& stream, const Robot& robot) {
        stream << robot.to_string();
        return stream;
    }

protected:
    Context* context_;

private:
    const int shell_;
    const bool self_;
};

/**
 * @brief A robot on our team
 * @details This extends Robot and provides methods for interacting with our
 * robots.
 * A few things this class is responsible for:
 * - specifying target position, velocity, angle, etc
 * - kicking and chipping the ball
 * - keeping track of which hardware revision this bot is
 * planner)
 */
class OurRobot : public Robot {
public:
    using RobotMask = std::array<float, kNumShells>;

    /**
     * @brief Construct a new OurRobot
     * @param context A pointer to the global system context object
     * @param shell The robot ID
     */
    OurRobot(Context* context, int shell);

    void add_status_text();

    void add_text(const QString& text, const QColor& qc = Qt::white,
                 const QString& layer_prefix = "RobotText");

    /// true if the kicker is ready
    bool charged() const;

    /// returns the time since the kicker was last charged, 0.0 if ready
    float kick_timer() const;

    /// segment for the location of the kicker
    rj_geometry::Segment kicker_bar() const;
    /// converts a point to the frame of reference of robot
    rj_geometry::Point point_in_robot_space(rj_geometry::Point pt) const;

    // simple checks to do geometry
    /** returns true if the position specified is behind the robot */
    // FIXME - Function name and comment don't match
    bool behind_ball(rj_geometry::Point ball_pos) const;

    // Constraints
    const planning::RobotConstraints& robot_constraints() const {
        return context_->robot_constraints[shell()];
    }

    planning::RobotConstraints& robot_constraints() { return context_->robot_constraints[shell()]; }

    const planning::MotionConstraints& motion_constraints() const {
        return robot_constraints().mot;
    }

    planning::MotionConstraints& motion_constraints() { return robot_constraints().mot; }

    /**
     * Returns a const reference to the path of the robot.
     */
    const planning::Trajectory& path() const { return context_->trajectories[shell()]; }

    /**
     * Returns a movable reference to the path of the robot.
     */
    planning::Trajectory&& path_movable() { return std::move(context_->trajectories[shell()]); }

    /// clears old radio_tx stuff, resets robot debug text, and clears local
    /// obstacles
    void reset_for_next_iteration();

    /// clears all fields in the robot's MotionConstraints object, causing the
    /// robot to stop
    void reset_motion_constraints();

    /** Stop the robot */
    void stop();

    /**
     * Makes this robot execute a line kick
     *
     * @param target - The target to kick towards (aiming point)
     */
    void line_kick(rj_geometry::Point target);

    /**
     * Intercept the ball as quickly as possible
     * May just slam into the ball if it does not have time to stop
     *
     * @param target - The target position to intercept the ball at
     */
    void intercept(rj_geometry::Point target);

    /**
     * @brief Move to a given point using the default RRT planner
     * @param end_speed - the speed we should be going when we reach the end of
     * the path
     */
    void move(rj_geometry::Point goal,
              rj_geometry::Point end_velocity = rj_geometry::Point());

    /**
     * @brief Move to a given point bypassing the RRT Path planner. This will
     * plan a direct path ignoring all obstacles and the starting velocity
     * @param end_speed - the speed we should be going when we reach the end of
     * the path
     */
    void move_direct(rj_geometry::Point goal, float end_speed = 0);

    /**
     * @brief Move to a given point while breaking everything. Tells the robot
     * it is already at the endpoint
     * @param end_speed - the speed we should be going when we reach the end of
     * the path. I'm not even sure if this part makes any sense here.
     */
    void move_tuning(rj_geometry::Point goal, float end_speed = 0);

    /**
     * @brief Move in front of the ball to intercept it. If a target face point
     * is given, the robot will try to face in that direction when the ball
     * hits.
     */
    void settle(std::optional<rj_geometry::Point> target);

    /**
     * @brief Approaches the ball and moves through it slowly
     */
    void collect();

    /*
     * Override the default angle planning strategy and face a point
     */
    void face(rj_geometry::Point pt);

    /**
     * Sets the world_velocity in the robot's MotionConstraints
     */
    void world_velocity(rj_geometry::Point target_world_vel);

    /**
     * The robot pivots around it's mouth toward the given target
     */
    void pivot(rj_geometry::Point pivot_target);

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
    void kick_level(uint8_t strength);

    /**
     * enable chip when ready at a given percentage of the currently set max
     * chip power.
     * @param strength a value between 0 and 1
     */
    void chip(float strength);

    /**
     * enable chip when ready at a given strength (0-255)
     */
    void chip_level(uint8_t strength);

    /**
     * @brief Undoes any calls to kick() or chip().
     */
    void unkick();

    RJ::Timestamp last_kick_time() const;

    const RobotStatus& radio_status() const {
        return context_->robot_status.at(shell());
    }

    /// checks if the bot has kicked/chipped very recently.
    bool just_kicked() const {
        return radio_status().kicker == RobotStatus::KickerState::kCharging;
    }

    /**
     * Gets a string representing the series of commands called on the robot
     * this iteration. Contains face(), move(), etc - used to display in the
     * BehaviorTree tab in soccer
     */
    std::string get_cmd_text() const;

    /**
     * ignore ball sense and kick immediately
     */
    void kick_immediately();

    /**
     * Adds an obstacle to the local set of obstacles for avoidance
     * Cleared after every frame
     */
    void local_obstacles(const std::shared_ptr<rj_geometry::Shape>& obs) {
        intent().local_obstacles.add(obs);
    }
    const rj_geometry::ShapeSet& local_obstacles() const {
        return intent().local_obstacles;
    }
    void clear_local_obstacles() { intent().local_obstacles.clear(); }

    rj_geometry::Point mouth_center_pos() const;

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
    bool has_ball() const;
    bool has_ball_raw() const;
    bool ball_sense_works() const;
    bool kicker_works() const;
    double kicker_voltage() const;
    RobotStatus::HardwareVersion hardware_version() const;

    const planning::MotionCommand& motion_command() const { return intent().motion_command; }
    void set_motion_command(const planning::MotionCommand& new_cmd) {
        if (intent().motion_command.index() != new_cmd.index()) {
            // clear path when command type changes
            context_->trajectories[shell()] = planning::Trajectory{{}};
        }

        intent().motion_command = new_cmd;
    }

    const planning::RotationConstraints& rotation_constraints() const {
        return robot_constraints().rot;
    }

    planning::RotationConstraints& rotation_constraints() { return robot_constraints().rot; }

    /**
     * @param age Time (in microseconds) that defines non-fresh
     */
    bool status_is_fresh(RJ::Seconds age = RJ::Seconds(0.5)) const;

    bool is_penalty_kicker = false;
    bool is_ball_placer = false;

    static void create_configuration(Configuration* cfg);

    double distance_to_chip_landing(int chip_power);
    uint8_t chip_power_for_distance(double distance);

    /**
     * Sets the priority which paths are planned.
     * Higher priority values are planned first.
     */
    void set_planning_priority(int8_t priority) { planning_priority_ = priority; }

    /**
     * Gets the priority which paths are planned.
     * Higher priority values are planned first.
     */
    int8_t get_planning_priority() const { return planning_priority_; }

    void set_pid(double p, double i, double d);

    void set_joystick_controlled(bool joystick_controlled);
    bool is_joystick_controlled() const;

protected:
    friend class Processor;
    friend class RadioNode;

    /// The processor mutates RadioRx in place and calls this afterwards to let
    /// it know that it changed
    void radio_rx_updated();

    const RobotLocalConfig* status() const {
        return &context_->local_configs[shell()];
    }

    const RobotConfig* config() const { return context_->robot_config.get(); }

private:
    RJ::Time last_ball_sense_;
    const RJ::Seconds lost_ball_duration_ = RJ::Seconds(0.1);

    void do_kick(uint8_t strength);
    void do_chip(uint8_t strength);
    void do_unkick();

    RobotStatus::KickerState last_kicker_status_ =
        RobotStatus::KickerState::kCharging;
    RJ::Time last_kick_time_;
    RJ::Time last_charged_time_;

    RobotIntent& intent() { return context_->robot_intents[shell()]; }
    const RobotIntent& intent() const {
        return context_->robot_intents[shell()];
    }

    /**
     * We build a string of commands such as face(), move(), etc at each
     * iteration
     * Then display this in the BehaviorTree tab in soccer
     */
    // note: originally this was not a pointer, but I got weird errors about a
    // deleted copy constructor...
    std::stringstream cmd_text_;

    void clear_cmd_text();

    static ConfigDouble* dribble_out_of_bounds_offset;

    int8_t planning_priority_{};
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
