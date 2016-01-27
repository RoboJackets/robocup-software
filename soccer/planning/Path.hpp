#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/ShapeSet.hpp>
#include <SystemState.hpp>
#include "MotionInstant.hpp"

#include <boost/optional.hpp>
#include <QColor>
#include <QString>

namespace Planning {

/**
 * @brief Abstract class representing a motion path
 */
class Path {
public:
    Path(RJ::Time startTime = 0) : _startTime(startTime) {}
    virtual ~Path() {}

    /**
     * This method evalates the path at a given time and returns the target
     * position and velocity of the robot.
     *
     * @param t Time (in seconds) since the robot started the path. Throws an
     *     exception if t<0
     * @return A MotionInstant containing the position and velocity at the given
     *     time if @t is within the range of the path.  If @t is not within the
     *     time range of this path, this method returns boost::none.
     */
    virtual boost::optional<MotionInstant> evaluate(float t) const = 0;

    /**
     * Returns true if the path hits an obstacle
     *
     * @param[in]	shape The obstacles on the field
     * @param[out]  hitTime the approximate time when the path hits an obstacle.
     * If no obstacles are hit, behavior is undefined for the final value.
     * @param[in] 	startTime The time on the path to start checking from
     * @return 		true if it hits an obstacle, otherwise false
     */
    virtual bool hit(const Geometry2d::ShapeSet& obstacles, float& hitTime,
                     float startTime) const = 0;

    /**
     * Draws the path.  The default implementation adds a DebugRobotPath to the
     * SystemState that interpolates points along the path.
     *
     * @param state The SystemState to draw the path on
     * @param color The color the path should be drawn
     * @param layer The layer to draw the path on
     */
    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const;

    /**
     * Returns how long it would take for the entire path to be traversed
     *
     * @return The time from start to path completion or infinity if it never
     * stops
     */
    virtual float getDuration() const = 0;

    /**
     * Returns a subPath
     *
     * @param startTime The startTime for from which the subPath should be
     *     taken.
     * @param endTime The endTime from which the subPath should be taken. If it
     *     is greater than the duration fo the path, it should go to the end of
     *     the path.
     * @return A unique_ptr to the new subPath
     */
    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const = 0;

    /// Start instant of the path
    virtual MotionInstant start() const = 0;
    /// Destination instant of the path
    virtual MotionInstant end() const = 0;

    /**
     * Returns a deep copy of the Path
     */
    virtual std::unique_ptr<Path> clone() const = 0;

    /// The time the path starts at
    RJ::Time startTime() const { return _startTime; }
    void setStartTime(RJ::Time t) { _startTime = t; }

protected:
    RJ::Time _startTime;
};

}  // namespace Planning
