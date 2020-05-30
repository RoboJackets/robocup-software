#pragma once

#include <Configuration.hpp>
#include <DebugDrawer.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <optional>
#include <planning/paths/Path.hpp>

namespace Planning {
/**
 * @brief Represents a motion path as a series of {pos, vel} pairs.
 *
 * @details The path represents a function of position given time that the robot
 *     should follow.  A line-segment-based path comes from the planner, then we
 *     use cubic bezier curves to interpolate and smooth it out.  This is done
 *     via the evaulate() method.
 */
class InterpolatedPath : public Path {
public:
    /// Each entry in InterpolatedPath is a MotionInstant and the time that the
    /// robot should be at that position and velocity.
    struct Entry {
        Entry(RobotInstant inst, RJ::Seconds t) : time(t) {
            if (inst.angle) {
                pose = Geometry2d::Pose(inst.motion.pos,
                                        inst.angle->angle.value_or(0));
                vel = Geometry2d::Twist(inst.motion.vel,
                                        inst.angle->angleVel.value_or(0));
            } else {
                pose = Geometry2d::Pose(inst.motion.pos, 0);
                vel = Geometry2d::Twist(inst.motion.vel, 0);
            }
        }

        Entry(Geometry2d::Pose pose, Geometry2d::Twist twist, RJ::Seconds t)
            : pose(pose), vel(twist), time(t) {}

        Entry(MotionInstant inst, RJ::Seconds t)
            : Entry(RobotInstant(inst), t) {}

        Geometry2d::Pose pose;
        Geometry2d::Twist vel;
        RJ::Seconds time;

        RobotInstant instant() const {
            // Create a new MotionInstant
            RobotInstant instant;
            instant.motion.pos = pose.position();
            instant.motion.vel = vel.linear();
            instant.angle = AngleInstant(pose.heading(), vel.angular());
            return instant;
        }
    };

    // Set of points in the path - used as waypoints
    std::vector<Entry> waypoints;

    /** default path is empty */
    InterpolatedPath() {}

    /** constructor with a single point */
    InterpolatedPath(RobotInstant p0);

    /** constructor from two points */
    InterpolatedPath(RobotInstant p0, RobotInstant p1);

    /**
     * \brief Constructor for when you have all the entries
     * @param entries
     */
    InterpolatedPath(std::vector<Entry>&& entries)
        : waypoints{std::move(entries)} {};

    /// Adds an instant at the end of the path for the given time.
    /// Time should not bet less than the last time.
    void addInstant(RJ::Seconds time, RobotInstant instant) {
        if (!waypoints.empty()) {
            assert(time > waypoints.back().time);
        }
        waypoints.push_back(Entry(instant, time));
    }

    // Overridden Path Methods
    virtual RobotInstant start() const override;
    virtual RobotInstant end() const override;
    virtual bool hit(const Geometry2d::ShapeSet& obstacles,
                     RJ::Seconds startTimeIntoPath,
                     RJ::Seconds* hitTime) const override;
    virtual std::unique_ptr<Path> subPath(
        RJ::Seconds startTime = RJ::Seconds::zero(),
        RJ::Seconds endTime = RJ::Seconds::max()) const override;
    virtual void draw(DebugDrawer* constdebug_drawer, const QColor& color,
                      const QString& layer) const override;
    virtual RJ::Seconds getDuration() const override;
    virtual std::unique_ptr<Path> clone() const override;

    [[nodiscard]] bool empty() const { return waypoints.empty(); }

    /// Erase all path contents
    void clear() { waypoints.clear(); }

    /**
     * Calulates the length of the path
     *
     * @param[in] start Index of point in path to use at start point.
     * @return the length of the path starting at point (start).
     */
    float length(unsigned int start = 0) const;

    /**
     * Calulates the length of the path
     *
     * @param[in] 	start Index of point in path to use at start point.
     * @param[in] 	end Index of point in path to use at end point.
     * @return the length of the path starting at point (start) and ending
     * at
     * point [end].
     */
    float length(unsigned int start, unsigned int end) const;

    /** Returns the length of the path from the closet point found to @a pt */
    float length(Geometry2d::Point pt) const;

    /** Returns number of waypoints */
    size_t size() const;

    // Returns the index of the point in this path nearest to pt.
    int nearestIndex(Geometry2d::Point pt) const;

    /** returns the nearest segement of @a pt to the path */
    Geometry2d::Segment nearestSegment(Geometry2d::Point pt) const;

    // Returns the shortest distance from this path to the given point
    float distanceTo(Geometry2d::Point pt) const;

    /**
     * Estimates how long it would take for the robot to get to a certain point
     * in the path using
     * Trapezoidal motion.
     *
     * @param index Index of the point on the path
     * @return The estimated time it would take for the robot to a point on the
     *     path starting from the start of the path
     */
    RJ::Seconds getTime(int index) const;

    static std::unique_ptr<Path> emptyPath(Geometry2d::Point position) {
        auto path = std::make_unique<InterpolatedPath>(
            RobotInstant(MotionInstant(position)));
        path->setDebugText("Empty Path");
        return std::move(path);
    }

protected:
    virtual std::optional<RobotInstant> eval(RJ::Seconds t) const override;
};

}  // namespace Planning
