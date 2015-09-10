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
    Path() : _startTime(0) {}
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
     * Draws the path
     *
     * @param[in]	state The SystemState to draw the path on
     * @param[in] 	color The color the path should be drawn
     * @param[in] 	layer The layer to draw the path on
     */
    virtual void draw(SystemState* const state, const QColor& color = Qt::black,
                      const QString& layer = "Motion") const = 0;

    /**
     * Returns how long it would take for the entire path to be traversed
     *
     * @return 	The time from start to path completion or infinity if it never
     * stops
     */
    virtual float getDuration() const = 0;

    /**
     * Returns a subPath
     *
     * @param[in]	startTime The startTime for from which the subPath should be
     taken.
     * @param[in] 	endTime The endTime from which the subPath should be taken.
     If it is greater than the duration fo the path,
                         it should go to the end of the path.
     * @return 	A unique_ptr to the new subPath
     */
    virtual std::unique_ptr<Path> subPath(
        float startTime = 0,
        float endTime = std::numeric_limits<float>::infinity()) const = 0;

    /// Endpoints of the path.  Will raise an InvalidPathException if the path
    /// is not valid.
    virtual MotionInstant start() const = 0;
    virtual MotionInstant end() const = 0;

    /**
     * Returns a deep copy of the Path
     */
    virtual std::unique_ptr<Path> clone() const = 0;

    /// The time the path starts at
    const Time startTime() const { return _startTime; }
    void setStartTime(Time t) { _startTime = t; }

protected:
    Time _startTime;
};

}  // namespace Planning
