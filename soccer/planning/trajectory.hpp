#pragma once

#include <rj_common/time.hpp>
#include <rj_geometry/pose.hpp>
#include <rj_msgs/msg/trajectory.hpp>

#include "debug_drawer.hpp"
#include "instant.hpp"
#include "planning/dynamic_obstacle.hpp"
#include "spdlog/spdlog.h"

namespace planning {

/**
 * @brief An ordered sequence (by time) of RobotInstants
 */
using RobotInstantSequence = std::vector<RobotInstant>;

/**
 * @brief Represents a trajectory x(t), y(t), h(t), with a smooth velocity and
 * piecewise-continuous acceleration.
 *
 * @details Note that this provides two interfaces: one accepting RJ::Seconds,
 * which is a duration, and one accepting RJ::Time, an instant in time.
 *
 * This uses cubic interpolation between two adjacent instants, fitting a
 * parametric cubic polynomial based on initial and final posiiton and velocity.
 *
 * A trajectory also carries a timestamp representing the time at which it was
 * planned. This should be filled in using `trajectory.stamp(RJ::Time)` before
 * publishing, and will be reset to `nullopt` when the trajectory is modified.
 */
class Trajectory {
public:
    using Msg = rj_msgs::msg::Trajectory;

    Trajectory() = default;
    ~Trajectory() = default;

    /**
     * @brief Create a trajectory from several "instants", each with a pose,
     * velocity, and timestamp. This
     */
    explicit Trajectory(RobotInstantSequence instants)
        : instants_(std::move(instants)) {}

    /**
     * @brief Create a trajectory from two other trajectories.
     *  Assumes a.last() == b.first() so b.first() is skipped.
     *
     * @details Is able to move out of the first trajectory, but not the second.
     */
    Trajectory(Trajectory a, const Trajectory& b);

    /**
     * Allow copy constructor, copy assignment, move constructor, and move
     * assignment
     */
    Trajectory(Trajectory&& other) noexcept = default;
    Trajectory(const Trajectory& other) = default;
    Trajectory& operator=(Trajectory&& other) noexcept = default;
    Trajectory& operator=(const Trajectory& other) = default;

    /**
     * @brief Append a RobotInstant. instant.stamp must be greater than end()
     *
     * @param instant The new RobotInstant to add.
     */
    void append_instant(RobotInstant instant);

    /**
     * @brief Hold the final position for a set time.
     *
     * Note that this will not hold forever. That would be bad, because
     * iterating over a trajectory would always hang.
     *
     * @param seconds the duration after the end of the trajectory to hold pose.
     */
    void hold_for(RJ::Seconds duration);

    /**
     * @brief Check that the given point in time is within bounds.
     */
    [[nodiscard]] bool check_time(RJ::Time time) const;

    /**
     * @brief Check that the given time is within bounds. Operates on a duration
     * (time into the path) rather than a timestamp.
     */
    [[nodiscard]] bool check_seconds(RJ::Seconds seconds) const;

    /**
     * @brief Contract or expand this trajectory by scaling velocities and
     * timestamps (relative to the beginning).
     *
     * @param final_duration The final duration of the trajectory.
     * @param fixed_point The time point that should remain unchanged. Defaults
     *  to the beginning of the trajectory.
     */
    void scale_duration(RJ::Seconds final_duration, RJ::Time fixed_point);

    /**
     * @copydoc Trajectory::ScaleDuration(RJ::Seconds, RJ::Time)
     */
    void scale_duration(RJ::Seconds final_duration);

    /**
     * @return The time point at the beginning of the trajectory.
     */
    [[nodiscard]] RJ::Time begin_time() const {
        if (instants_.empty()) {
            return RJ::now();
        }

        return first().stamp;
    }

    /**
     * @return The time point at the end of the trajectory.
     */
    [[nodiscard]] RJ::Time end_time() const {
        if (instants_.empty()) {
            return RJ::now();
        }

        return last().stamp;
    }

    /**
     * @return The duration of the trajectory.
     */
    [[nodiscard]] RJ::Seconds duration() const {
        return end_time() - begin_time();
    }

    /**
     * @brief Evaluate this trajectory (calculate position and velocity) at a
     *  given point in time.
     *
     * @note When iterating over a trajectory, prefer the cursor interface.
     *
     * @param time The time point to evaluate at (absolute time, not a duration
     *      into the path)
     * @return The RobotInstant at that point, or nullopt if time is
     *      out-of-bounds.
     */
    [[nodiscard]] std::optional<RobotInstant> evaluate(RJ::Time time) const;

    /**
     * @brief Evaluate this trajectory (calculate position and velocity) at a
     *  given duration past the beginning of the trajectory.
     *
     * @note When iterating over a trajectory, prefer the cursor interface.
     *
     * @param seconds The duration since the beginning of the path.
     * @return The RobotInstant at that duration into the path, or nullopt if
     *      seconds is out-of-bounds.
     */
    [[nodiscard]] std::optional<RobotInstant> evaluate(
        RJ::Seconds seconds) const;

    /**
     * @brief Returns a trajectory formed using an interval subset of this
     *  trajectory.
     *
     * @param clip_start_time The time from which the sub-trajectory should start.
     * @param clip_end_time The time at which the sub-trajectory should end.
     *  If it is after the trajectory's end, the sub-trajectory will be taken
     *  to the end of the trajectory.
     *
     * @return a sub-trajectory of the original trajectory.
     */
    [[nodiscard]] Trajectory sub_trajectory(RJ::Time clip_start_time,
                                           RJ::Time clip_end_time) const;

    /**
     * @return the instant count. Intended for use when editing a trajectory
     *  in-place.
     */
    [[nodiscard]] int num_instants() const { return instants_.size(); }

    /**
     * @brief Get a reference to the ith instant.
     *
     * @param i the instant index to retrieve. Must be in range
     *  [0, num_instants())
     * @return a reference to the ith instant.
     *
     * @details Intended for use when editing a trajectory in-place; for other
     *  cases it may be simpler to use the iterator or Cursor interfaces.
     */
    [[nodiscard]] RobotInstant& instant_at(int i) { return instants_.at(i); }

    /**
     * @copydoc Trajectory::instant_at()
     */
    [[nodiscard]] RobotInstant instant_at(int i) const {
        return instants_.at(i);
    }

    /**
     * @brief Iterator interface. Should only be used for working with STL
     *  functions, not for iterating over trajectories.
     */
    [[nodiscard]] auto instants_end() { return instants_.end(); }
    /**
     * @copydoc Trajectory::instants_end()
     */
    [[nodiscard]] auto instants_end() const { return instants_.cend(); }
    /**
     * @copydoc Trajectory::instants_end()
     */
    [[nodiscard]] auto instants_begin() {
        if (instants_.empty() || instants_.begin() == instants_.end()) {
            throw std::runtime_error("instants_ is empty, this cannot work!");
        }
        return instants_.begin();
    }
    /**
     * @copydoc Trajectory::instants_end()
     */
    [[nodiscard]] auto instants_begin() const {
        if (instants_.empty() || instants_.begin() == instants_.end()) {
            throw std::runtime_error("instants_ is empty, this cannot work!");
        }
        return instants_.cbegin();
    }

    /**
     * @brief Check if this is an empty path.
     */
    [[nodiscard]] bool empty() const { return instants_.empty(); }

    /**
     * @brief Check whether all poses in two trajectories are nearly equal.
     */
    static bool nearly_equal(const Trajectory& a, const Trajectory& b,
                             double tolerance = 1e-6);

    /**
     * @brief Get the first instant in the past, or crash if the path is empty.
     * @return The first instant in the path.
     */
    RobotInstant& first() {
        if (empty()) {
            throw std::runtime_error(
                "Can't get the first() instant in an empty trajectory");
        }
        return instants_.front();
    }

    /**
     * @copydoc Trajectory::first()
     */
    [[nodiscard]] const RobotInstant& first() const {
        if (empty()) {
            throw std::runtime_error(
                "Can't get the first() instant in an empty trajectory");
        }
        return instants_.front();
    }

    /**
     * Get the last instant in the past, or crash if the path is empty.
     * @return The last instant in the path.
     */
    RobotInstant& last() {
        if (empty()) {
            throw std::runtime_error(
                "Can't get the last() instant in an empty trajectory");
        }
        return instants_.back();
    }

    [[nodiscard]] const RobotInstant& last() const {
        if (empty()) {
            throw std::runtime_error(
                "Can't get the last() instant in an empty trajectory");
        }
        return instants_.back();
    }

    /**
     * Draw this trajectory.
     * @param drawer The debug drawer to use.
     * @param alt_text_position Optional position at which to draw the text in
     * the event that the trajectory is empty.
     */
    void draw(DebugDrawer* drawer,
              std::optional<rj_geometry::Point> alt_text_position =
                  std::nullopt) const;

    /**
     * Interpolate between two robot instants
     * @param prev previous robot instant
     * @param next next robot instant
     * @param time time between previous and next
     * @return the interpolated robot instant at the given time
     */
    static RobotInstant interpolated_instant(const RobotInstant& prev,
                                            const RobotInstant& next,
                                            RJ::Time time);

    void set_debug_text(std::string str) { debug_text_ = std::move(str); };
    [[nodiscard]] std::string get_debug_text() const {
        return debug_text_ ? *debug_text_ : "";
    }

    /**
     * @brief Get the time this trajectory was created, or nullopt if it has not
     * yet been stamped.
     */
    [[nodiscard]] std::optional<RJ::Time> time_created() const { return creation_stamp_; }

    /**
     * @brief Stamp this trajectory for completion at the specified time.
     */
    void stamp(RJ::Time time) { creation_stamp_ = time; }

    /*
     * @brief Allows seeking to arbitrary positions in a Trajectory and
     *  incrementing by arbitrary steps.
     */
    class Cursor {
    public:
        /**
         * @brief Construct a cursor from the given trajectory and start time.
         */
        Cursor(const Trajectory& trajectory, RJ::Time start_time);

        /**
         * @brief Construct a cursor at the beginning of the given trajectory.
         */
        Cursor(const Trajectory& trajectory);

        /**
         * @brief Construct a cursor starting at a given iterator.
         */
        Cursor(const Trajectory& trajectory,
               RobotInstantSequence::const_iterator iterator);

        /**
         * @brief Whether or not there exists an instant at the time specified
         *  by this cursor.
         */
        [[nodiscard]] bool has_value() const {
            return trajectory_.check_time(time_);
        }

        /**
         * @brief Evaluate the trajectory at the time specified by this cursor.
         */
        [[nodiscard]] RobotInstant value() const;

        /**
         * @brief Set this cursor to an arbitrary point in time.
         * @details Operates in O(log n) average case with n points in the
         *  trajectory.
         */
        void seek(RJ::Time time);

        /**
         * @brief Advance this cursor (forwards only) by a particular increment,
         *  in seconds. Reverse advancing (negative arguments) not supported.
         *
         * @details Operates in linear time with the number of knot points
         *  forward to jump (larger increments will mean longer runtime).
         *  If this steps to after the end of the trajectory, this cursor will
         *  not point to a value.
         */
        void advance(RJ::Seconds seconds);

        /**
         * @brief Move this cursor to the next knot point. If it is already at a
         *  knot, move to the next one. If it is in an interval between a and b,
         *  move to b. If it is at the end, it will move off of the trajectory.
         */
        void next_knot();

        /**
         * @return The current time associated with this cursor. Not guaranteed
         *  to be a valid time within the trajectory's bounds.
         */
        [[nodiscard]] RJ::Time time() const { return time_; }

        /**
         * @return The time associated with the knot point immediately after
         *  this cursor, or nullopt if there is no such knot (this is the end).
         */
        [[nodiscard]] std::optional<RJ::Time> time_next() const {
            if (iterator_ == trajectory_.instants_end() ||
                iterator_ + 1 == trajectory_.instants_end()) {
                return std::nullopt;
            }
            return (iterator_ + 1)->stamp;
        }

    private:
        const Trajectory& trajectory_;

        // An iterator pointing to the knot point at or immediately before the
        // time point referred to by `time_`.
        RobotInstantSequence::const_iterator iterator_;
        RJ::Time time_;
    };

    /**
     * @brief Get a Cursor into this trajectory. Useful for iterating over it.
     */
    [[nodiscard]] Cursor cursor(RJ::Time start_time) const;

    /**
     * @brief Get a cursor to the beginning of the trajectory.
     */
    [[nodiscard]] Cursor cursor_begin() const;

    /**
     * Mark this trajectory as having a valid angle profile.
     *
     * This will be reset upon the following conditions:
     *  - New instants are appended
     *  - The trajectory is concatenated with another trajectory.
     */
    void mark_angles_valid() { has_angle_profile_ = true; }

    /**
     * @return True when this trajectory is properly angle profiled.
     */
    [[nodiscard]] bool angles_valid() const { return has_angle_profile_; }

    /**
     * @return The robot instants forming the keypoints of this trajectory, sequenced by time.
     */
    [[nodiscard]] const RobotInstantSequence& instants() const { return instants_; }

private:
    // A sorted array of RobotInstants (by timestamp)
    RobotInstantSequence instants_;

    // Time this trajectory was created. If the trajectory is not yet finished,
    // this will be std::nullopt.
    std::optional<RJ::Time> creation_stamp_;

    std::optional<std::string> debug_text_;

    // Whether or not this has been profiled.
    bool has_angle_profile_{false};
};

bool operator==(const Trajectory& a, const Trajectory& b);

}  // namespace planning

namespace rj_convert {

template <>
struct RosConverter<planning::Trajectory, rj_msgs::msg::Trajectory> {
    static rj_msgs::msg::Trajectory to_ros(const planning::Trajectory& from) {
        if (!from.angles_valid()) {
            throw std::invalid_argument("Cannot serialize trajectory with invalid angles");
        }
        return rj_msgs::build<rj_msgs::msg::Trajectory>()
            .stamp(convert_to_ros(from.time_created().value()))
            .instants(convert_to_ros(from.instants()));
    }

    static planning::Trajectory from_ros(const rj_msgs::msg::Trajectory& from) {
        auto trajectory = planning::Trajectory{convert_from_ros(from.instants)};
        trajectory.stamp(convert_from_ros(from.stamp));
        trajectory.mark_angles_valid();
        return trajectory;
    }
};

ASSOCIATE_CPP_ROS(planning::Trajectory, planning::Trajectory::Msg);

}  // namespace rj_convert
