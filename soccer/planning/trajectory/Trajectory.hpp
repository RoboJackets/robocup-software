#pragma once

#include <Geometry2d/Pose.hpp>
#include <time.hpp>
#include "WorldState.hpp"
#include "DebugDrawer.hpp"
#include "planning/DynamicObstacle.hpp"

namespace Planning {

/**
 * Represents the current state of a robot in a planned trajectory.
 */
struct RobotInstant {
    RobotInstant(Geometry2d::Pose pose, Geometry2d::Twist velocity, RJ::Time stamp)
        : pose(pose), velocity(velocity), stamp(stamp) {}

    RobotInstant() = default;

    Geometry2d::Pose pose;
    Geometry2d::Twist velocity;
    RJ::Time stamp;

    /**
     * Equality comparison operator.
     */
    bool operator==(const RobotInstant& other) const {
        return pose == other.pose && velocity == other.velocity && stamp == other.stamp;
    }

    /**
     * Inequality comparison operator.
     */
    bool operator!=(const RobotInstant& other) const {
        return pose != other.pose || velocity != other.velocity || stamp != other.stamp;
    }
};

/**
 * Represents a trajectory x(t), y(t), h(t), with a smooth velocity and
 * piecewise-continuous acceleration.
 *
 * Note that this provides two interfaces: one accepting RJ::Seconds, which is
 * a duration, and one accepting RJ::Time, an instant in time.
 *
 * This uses cubic interpolation between two adjacent instants, fitting a
 * parametric cubic polynomial based on initial and final posiiton and velocity.
 */
class TrajectoryIterator;
//todo(Ethan) implement move-constructor, move-assign, copy-constructor, copy-assign
class Trajectory {
public:
    /**
     * Create a trajectory from several "instants", each with a pose, velocity,
     * and timestamp.
     */
    explicit Trajectory(std::vector<RobotInstant> &&instants) : instants_(std::move(instants)) {}

    /**
     * Create a trajectory from two other trajectories
     * assumes a.last() == b.first() so b.first() is skipped
     * This copies all old RobotInstants, so don't use this with too many points
     * (see CompositePath for a more efficient way if needed)
     */
    Trajectory(const Trajectory& a, const Trajectory& b);
    Trajectory(Trajectory&& a, Trajectory&& b);

    /**
     * allow copy constructor, copy assignment, move constructor, and move assignment
     */
    Trajectory(Trajectory&& other): instants_(std::move(other.instants_)), _debugText(std::move(other._debugText)), angle_override(std::move(other.angle_override)) {
        other.instants_ = std::vector<RobotInstant>{};
        other.angle_override = std::nullopt;
        other._debugText = "MOVED FROM";
    }
    Trajectory(const Trajectory& other) = default;
    Trajectory& operator=(Trajectory&& other);
    Trajectory& operator=(const Trajectory& other);

    /**
     * Insert a RobotInstant based on its timestamp.
     *
     * @param instant The new RobotInstant to add.
     */
    void InsertInstant(RobotInstant instant);

    /**
     * Append a RobotInstant. instant.stamp must be greater than end()
     *
     * @param instant The new RobotInstant to add.
     */
    void AppendInstant(RobotInstant instant);

    /**
     * Check that the given time is within bounds. Operates on a timestamp,
     * not a delta time.
     */
    bool CheckTime(RJ::Time time) const;

    /**
     * Check that the given time is within bounds. Operates on a duration (time
     * into the path) rather than a timestamp.
     */
    bool CheckSeconds(RJ::Seconds seconds) const;

    /**
     * Returns true if the path hits an obstacle
     *
     * @param[in]	shape The obstacles on the field
     * @param[out]  hitTime the approximate time when the path hits an obstacle.
     * If no obstacles are hit, behavior is undefined for the final value.
     * @param[in] 	startTimeIntoPath The time on the path to start checking
     *from
     * @return 		true if it hits an obstacle, otherwise false
     */
    bool hit(const Geometry2d::ShapeSet& obstacles, RJ::Seconds startTimeIntoPath, RJ::Seconds* hitTime) const;

    bool intersects(const std::vector<DynamicObstacle>& obstacles,
                    RJ::Time startTime,
                    Geometry2d::Point* hitLocation,
                    RJ::Seconds* hitTime) const;

    /**
     * Contract or expand this trajectory by scaling velocities and timestamps
     * (relative to the beginning).
     *
     * @param final_duration The final duration of the trajectory.
     * @param fixed_point The time point that should remain unchanged. Defaults
     *      to the beginning of the trajectory.
     */
    void ScaleDuration(RJ::Seconds final_duration, RJ::Time fixed_point);

    void ScaleDuration(RJ::Seconds final_duration);

    /**
     * @return The time point at the beginning of the trajectory.
     */
    RJ::Time begin_time() const {
        if (instants_.empty()) {
            return RJ::now();
        } else {
            return instants_.front().stamp;
        }
    }

    /**
     * @return The time point at the end of the trajectory.
     */
    RJ::Time end_time() const {
        if (instants_.empty()) {
            return RJ::now();
        } else {
            return instants_.back().stamp;
        }
    }

    /**
     * @return The duration of the trajectory.
     */
    RJ::Seconds duration() const {
        return end_time() - begin_time();
    }

    /**
     * Evaluate this trajectory (calculate position and velocity) at a given
     * point in time.
     *
     * @param time The time point to evaluate at (absolute time, not a duration
     *      into the path)
     * @return The RobotInstant at that point, or nullopt if time is
     *      out-of-bounds.
     */
    std::optional<RobotInstant> evaluate(RJ::Time time) const;

    /**
     * Evaluate this trajectory (calculate position and velocity) at a given
     * duration past the beginning of the trajectory.
     *
     * @param seconds The duration since the beginning of the path.
     * @return The RobotInstant at that duration into the path, or nullopt if
     *      seconds is out-of-bounds.
     */
    std::optional<RobotInstant> evaluate(RJ::Seconds seconds) const;

    /**
     * Returns a subTrajectory
     *
     * @param startTime The startTime for from which the subTrajectory should be
     *     taken.
     * @param endTime The endTime from which the subTrajectory should be taken. If it
     *     is greater than the duration fo the trajectory, it should go to the end of
     *     the trajectory.
     * @return a subTrajectory
     */
    Trajectory subTrajectory(RJ::Seconds startTime, RJ::Seconds endTime) const;

    /**
     * Get the instant count. Intended for use when editing a trajectory in-
     * place.
     *
     * @return
     */
    int num_instants() const { return instants_.size(); }

    /**
     * Get a reference to the specified index. Intended for use when editing
     * a trajectory in-place.
     * @param index The index of the desired instant. The relation
     *      0 <= index <= num_instants() must hold.
     * @return The instant at the given index.
     */
    RobotInstant &instant(int index) { return instants_.at(index); }

    /**
     * Check if this is an empty path.
     */
    bool empty() const { return instants_.empty(); }

    /**
     * Get the first instant in the past, or crash if the path is empty.
     * @return The first instant in the path.
     */
    RobotInstant first() const { return instants_.front(); }

    /**
     * Get the last instant in the past, or crash if the path is empty.
     * @return The last instant in the path.
     */
    RobotInstant last() const { return instants_.back(); }

    /**
     * Draw this trajectory.
     * @param drawer The debug drawer to use.
     */
    void draw(DebugDrawer* drawer, std::optional<Geometry2d::Point> backupPos = std::nullopt) const;

    /**
     * make a clone
     * @return clone
     */
    Trajectory clone() const {
        return Trajectory(std::vector<RobotInstant>(instants_));
    }

    /**
     * get an iterator
     * @param startTime start time
     * @param deltaT time step
     * @return iterator
     */
    TrajectoryIterator iterator(RJ::Time startTime, RJ::Seconds deltaT) const;

    void setDebugText(QString str) {_debugText = std::move(str); };
    QString getDebugText() { return _debugText ? *_debugText: ""; }

protected:
    // A sorted array of RobotInstants (by timestamp)
    std::vector<RobotInstant> instants_;

    //a super hacky way to force a change of angle/heading
    std::optional<double> angle_override;

    std::optional<QString> _debugText;
};

class TrajectoryIterator {
public:
    explicit TrajectoryIterator(const Trajectory& trajectory, RJ::Time startTime, RJ::Seconds deltaT)
            : _trajectory(trajectory), _time(startTime - trajectory.begin_time()), _deltaT(deltaT) {}

    virtual RobotInstant operator*() const {
        auto temp = _trajectory.evaluate(_time);
        if (temp) {
            return *temp;
        } else {
            return _trajectory.last();
        }
    }

    virtual TrajectoryIterator& operator++() {
        _time += _deltaT;
        return *this;
    }

private:
    const Trajectory& _trajectory;
    RJ::Seconds _time;
    const RJ::Seconds _deltaT;
};

} // namespace Planning
