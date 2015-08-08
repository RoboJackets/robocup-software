#pragma once
#include <Geometry2d/Point.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <SystemState.hpp>
#include <QColor>
#include <QString>
#include "MotionInstant.hpp"

namespace Planning
{
	/**
	 * @brief Abstract class representing a motion path
	 */
	class Path
	{

	public:
        virtual ~Path() {}

		/**
         * A path describes the position and velocity a robot should be at for a
         * particular time interval.  This method evalates the path at a given time and
         * returns the target position and velocity of the robot.
         *
         * @param[in] 	t Time (in seconds) since the robot started the path. Throws an exception if t<0
         * @param[out] 	targetMotionInstant The position and velocity the robot would ideally be at at the given time
         * @return 		true if the path is valid at time @t, false if you've gone past the end
         */
		virtual bool evaluate(float t, MotionInstant &targetMotionInstant) const=0;

		/**
		 * Returns true if the path hits an obstacle
		 *
		 * @param[in]	shape The obstacles on the field
		 * @param[out]  hitTime the approximate time when the path hits an obstacle. If no obstacles are hit, behavior is undefined for the final value.
		 * @param[in] 	startTime The time on the path to start checking from
		 * @return 		true if it hits an obstacle, otherwise false
		 */
		virtual bool hit(const Geometry2d::CompositeShape &shape, float &hitTime, float startTime) const=0;

		/**
		 * Draws the path
		 *
		 * @param[in]	state The SystemState to draw the path on
		 * @param[in] 	color The color the path should be drawn
		 * @param[in] 	layer The layer to draw the path on
		 */
		virtual void draw(SystemState * const state, const QColor &color = Qt::black, const QString &layer = "Motion") const =0;

		/**
		 * Returns how long it would take for the entire path to be traversed
		 *
		 * @return 	The time from start to path completion or infinity if it never stops
		 */
		virtual float getDuration() const=0;

		/**
		 * Returns a subPath
		 *
		 * @param[in]	startTime The startTime for from which the subPath should be taken.
		 * @param[in] 	endTime The endTime from which the subPath should be taken. If it is greater than the duration fo the path,
							 it should go to the end of the path.
		 * @return 	A unique_ptr to the new subPath
		 */
		virtual std::unique_ptr<Path> subPath(float startTime = 0, float endTime = std::numeric_limits<float>::infinity()) const=0;

		/**
		 * Returns the destination point of the path if it has one
		 */
		virtual boost::optional<MotionInstant> destination() const=0;

		/**
		 * Returns a deep copy of the Path
		 */
		virtual std::unique_ptr<Path> clone() const=0;
	};
}
