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
			 * @param[in] 	startTime The time in the path from which to check
			 * @return 		true if the path is valid, false if it hits an obstacle
			 */
			virtual bool hit(const Geometry2d::CompositeShape &shape, float startTime = 0) const=0;

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
			virtual boost::optional<Geometry2d::Point> destination() const=0;
	};
}