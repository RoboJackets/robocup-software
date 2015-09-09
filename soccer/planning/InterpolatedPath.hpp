#pragma once

#include <planning/Path.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <Configuration.hpp>

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
        Entry(MotionInstant inst, float t) : instant(inst), time(t) {}

        MotionInstant instant;
        float time;

        Geometry2d::Point& pos() { return instant.pos; }
        const Geometry2d::Point& pos() const { return instant.pos; }

        Geometry2d::Point& vel() { return instant.vel; }
        const Geometry2d::Point& vel() const { return instant.vel; }
    };

    // Set of points in the path - used as waypoints
    std::vector<Entry> waypoints;

    /** default path is empty */
    InterpolatedPath() {}

    /** constructor with a single point */
    InterpolatedPath(Geometry2d::Point p0);

    /** constructor from two points */
    InterpolatedPath(Geometry2d::Point p0, Geometry2d::Point p1);

    /// Adds an instant at the end of the path for the given time.
    /// Time should not bet less than the last time.
    void addInstant(float time, MotionInstant instant) {
        if (!waypoints.empty()) {
            assert(time > waypoints.back().time);
        }
        waypoints.push_back(Entry(instant, time));
    }

    // Overridden Path Methods
    virtual bool valid() const override { return !waypoints.empty(); }
    virtual MotionInstant start() const override;
    virtual MotionInstant end() const override;
    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float startTime) const override;
    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const override;
    virtual void draw(SystemState* const state, const QColor& color,
                      const QString& layer) const override;
    virtual boost::optional<MotionInstant> evaluate(float t) const override;
    virtual float getDuration() const override;
    virtual std::unique_ptr<Path> clone() const override;

    bool empty() const { return waypoints.empty(); }

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
    float getTime(int index) const;
};

}  // namespace Planning
