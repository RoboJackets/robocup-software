#pragma once

#include <Geometry2d/Pose.hpp>
#include <time.hpp>
#include "WorldState.hpp"
#include "DebugDrawer.hpp"
#include "planning/DynamicObstacle.hpp"
#include "Utils.hpp"

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
class Trajectory {
public:
    /**
     * Create a trajectory from several "instants", each with a pose, velocity,
     * and timestamp.
     */
    explicit Trajectory(std::list<RobotInstant> &&instants) : instants_(std::move(instants)) {}

    /**
     * Create a trajectory from two other trajectories
     * assumes a.last() == b.first() so b.first() is skipped
     * This copies all old RobotInstants, so don't use this with too many points
     * (see CompositePath for a more efficient way if needed)
     * Time Complexity: Constant!
     */
    Trajectory(const Trajectory& a, const Trajectory& b);
    Trajectory(Trajectory&& a, Trajectory&& b);

    /**
     * allow copy constructor, copy assignment, move constructor, and move assignment
     */
    Trajectory(Trajectory&& other);
    Trajectory(const Trajectory& other) = default;
    Trajectory& operator=(Trajectory&& other);
    Trajectory& operator=(const Trajectory& other);

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
    bool hit(const Geometry2d::ShapeSet& obstacles, RJ::Seconds startTimeIntoPath, RJ::Seconds* hitTime = nullptr) const;

    bool intersects(const std::vector<DynamicObstacle>& obstacles,
                    RJ::Time startTime,
                    Geometry2d::Point* hitLocation = nullptr,
                    RJ::Seconds* hitTime = nullptr) const;

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
            return first().stamp;
        }
    }

    /**
     * @return The time point at the end of the trajectory.
     */
    RJ::Time end_time() const {
        if (instants_.empty()) {
            return RJ::now();
        } else {
            return last().stamp;
        }
    }

    /**
     * @return The duration of the trajectory.
     */
    RJ::Seconds duration() const {
        return end_time() - begin_time();
    }

    /** todo(Ethan) delete these functions bc they are slow
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
     * Complexity: O(n)
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
     * Delete instants in the trajectory before startTime and add a
     * new intermediate point to the front if necessary.
     *
     * @param startTime start time
     */
    void trimFront(RJ::Seconds startTime);

    /**
     * Get the instant count. Intended for use when editing a trajectory in-
     * place.
     *
     * @return
     */
    int num_instants() const {
        if(empty()) {
            return 0;
        }
        int numPost = post_instants ? post_instants->size() : 0;
        return instants_.size() + numPost;
    }


    class InstantsIterator;
    InstantsIterator instants_begin();
    InstantsIterator instants_begin() const;
    InstantsIterator instants_end();
    InstantsIterator instants_end() const;

    /**
     * Check if this is an empty path.
     */
    bool empty() const { return instants_.empty(); }

    /**
     * Get the first instant in the past, or crash if the path is empty.
     * @return The first instant in the path.
     */
    RobotInstant& first() {
        assert(!instants_.empty());
        return instants_.front();
    }
    const RobotInstant& first() const {
        assert(!instants_.empty());
        return instants_.front();
    }

    /**
     * Get the last instant in the past, or crash if the path is empty.
     * @return The last instant in the path.
     */
    RobotInstant& last() {
        *raw_end_iterator();
    }
    const RobotInstant& last() const {
        *raw_end_iterator();
    }

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
        return Trajectory(std::list<RobotInstant>(instants_));
    }

    /**
     * get an iterator
     * @param startTime start time
     * @param deltaT time step
     * @return iterator
     */
    TrajectoryIterator iterator(RJ::Time startTime, RJ::Seconds deltaT) const;

    /**
     * Interpolate between two robot instants
     * @param prev previous robot instant
     * @param next next robot instant
     * @param time time between previous and next
     * @return the interpolated robot instant at the given time
     */
    static RobotInstant interpolatedInstant(const RobotInstant& prev, const RobotInstant& next, RJ::Time time);

    void setDebugText(QString str) {_debugText = std::move(str); };
    QString getDebugText() { return _debugText ? *_debugText: ""; }

private:
    // A sorted array of RobotInstants (by timestamp)
    std::list<RobotInstant> instants_;
    // extra instants to be used after instants_ ends.
    std::optional<std::list<RobotInstant>> post_instants;

    // force a change of angle/heading
    std::optional<double> angle_override;

    std::optional<QString> _debugText;

    std::list<RobotInstant>::iterator raw_end_iterator() {
        assert(!empty());
        if(post_instants && !post_instants->empty()) {
            return post_instants->end();
        }
        return instants_.end();
    }
    std::list<RobotInstant>::const_iterator raw_end_iterator() const {
        assert(!empty());
        if(post_instants && !post_instants->empty()) {
            return post_instants->end();
        }
        return instants_.end();
    }
};

/*
 * This is a wrapper for std::list<RobotIntant>::iterator to account for
 * the two different lists instants_ and post_instants
 */
class Trajectory::InstantsIterator {
public:
    InstantsIterator(const Trajectory& trajectory, const std::list<RobotInstant>::iterator& iterator): _trajectory(&trajectory), _iterator(iterator) {}
    InstantsIterator& operator++();
    InstantsIterator operator++(int) {
        InstantsIterator tmp = *this;
        ++(*this);
        return tmp;
    }
    InstantsIterator& operator--();
    InstantsIterator operator--(int) {
        InstantsIterator tmp = *this;
        --(*this);
        return tmp;
    }
    bool operator==(const InstantsIterator& other) const {
        return _trajectory == other._trajectory && _iterator == other._iterator;
    };
    bool operator!=(const InstantsIterator& other) const {
        return !(*this == other);
    }
    RobotInstant& operator*() {
        assert(hasValue());
        return *_iterator;
    }
    std::list<RobotInstant>::iterator operator->() {
        assert(hasValue());
        return _iterator;
    }
    InstantsIterator peekNext() const {
        assert(hasNext());
        InstantsIterator nextIt = *this;
        ++nextIt;
        return nextIt;
    }
    bool hasValue() const {
        return _iterator != _trajectory->instants_.end() && !(_trajectory->post_instants && _iterator == _trajectory->post_instants->end());
    }
    bool hasNext() const {
        if(!hasValue()) return false;
        InstantsIterator nxt = *this;
        ++nxt;
        return nxt.hasValue();
    }

private:
    const Trajectory* _trajectory;
    std::list<RobotInstant>::iterator _iterator;
};

/*
 * Interpolates over a trajectory in equal time steps.
 */
class TrajectoryIterator {
public:
    TrajectoryIterator(const Trajectory& trajectory, RJ::Time startTime, RJ::Seconds deltaT);

    RobotInstant operator*() const;

    TrajectoryIterator& operator++() {
        advance(_time + _deltaT);
        return *this;
    }

    RobotInstant peekNext() const {
        auto it = *this;
        return *++it;
    }

    bool hasNext() const {
        return _trajectory.CheckTime(_time + _deltaT);
    }
    bool hasValue() const {
        return _trajectory.CheckTime(_time);
    }

    void advance(RJ::Time time);

private:
    const Trajectory& _trajectory;
    Trajectory::InstantsIterator _iterator;
    RJ::Time _time;
    const RJ::Seconds _deltaT;
};

} // namespace Planning
