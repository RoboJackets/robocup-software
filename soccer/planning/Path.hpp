#pragma once
#include <Geometry2d/Point.hpp>
#include <Geometry2d/CompositeShape.hpp>
#include <SystemState.hpp>
#include <QColor>
#include <QString>

namespace Planning
{
	/**
	 * @brief Abstract class representing a motion path
	 */
	class Path {
		public:

			/**
			 * A path describes the position and velocity a robot should be at for a
			 * particular time interval.  This method evalates the path at a given time and
			 * returns the target position and velocity of the robot.
			 *
			 * @param[in] 	t Time (in seconds) since the robot started the path
			 * @param[out] 	targetPosOut The position the robot would ideally be at at the given time
			 * @param[out] 	targetVelOut The target velocity of the robot at the given time
			 * @return 		true if the path is valid at time @t, false if you've gone past the end
			 */
			virtual bool evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const=0;

			/**
			 * Returns true if the path never touches an obstacle or additionally, when exitObstacles is true, if the path
			 * starts out in an obstacle but leaves and never re-enters any obstacle.
			 *
			 * @param[in]	shape The obstacles on the field
			 * @param[in] 	start The point on the path to start checking from
			 * @return 		true if the path is valid, false if it hits an obstacle
			 */
			virtual bool hit(const Geometry2d::CompositeShape &shape, Geometry2d::Point *startPosition = NULL) const=0;

			/**
			 * Draws the path
			 *
			 * @param[in]	state The SystemState to draw the path on
			 * @param[in] 	color The color the path should be drawn
			 * @param[in] 	layer The layer to draw the path on
			 */
			virtual void draw(SystemState * const state, const QColor &color = Qt::black, const QString &layer = "Motion") const =0;
			
			/** 
			 * Estimates how long it would take for the robot to traverse the entire path
			 *
			 * @return 	The time from start to path completion or -1 if there is no destination
			 */
			virtual float getTime() const=0;

			/**
			 * Returns the destination point of the path if it has one
			 */
			virtual boost::optional<Geometry2d::Point> destination() const=0;
	};
}