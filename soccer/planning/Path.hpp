
#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include "Obstacle.hpp"

namespace Planning
{
	/**
	 *	Represents a path as a series of 2d points
	 */
	class Path
	{
		public:

			/** default path is empty */
			Path() {}

			/** constructor with a single point */
			Path(const Geometry2d::Point& p0);

			/** constructor from two points */
			Path(const Geometry2d::Point& p0, const Geometry2d::Point& p1);

			bool empty() const
			{
				return points.empty();
			}
			
			void clear()
			{
				points.clear();
			}
			
			// Returns the length of the path starting at point (start).
			float length(unsigned int start = 0) const;
			
			/** returns the length of the path from the closet point found to @a pt */
			float length(const Geometry2d::Point &pt) const;
			
			// number of waypoints
			size_t size() const { return points.size(); }

			/** returns true if the path has non-zero size */
			bool valid() const { return !points.empty(); }

			// Returns the index of the point in this path nearest to pt.
			int nearestIndex(const Geometry2d::Point &pt) const;
			
			/** returns the nearest segement of @a pt to the path */
			Geometry2d::Segment nearestSegment(const Geometry2d::Point &pt) const;
			
			// Returns the shortest distance from this path to the given point
			float distanceTo(const Geometry2d::Point &pt) const;
			
			// Returns the start of the path
			Geometry2d::Point::Optional start() const;

			// Returns a new path starting from a given point
			void startFrom(const Geometry2d::Point& pt, Planning::Path& result) const;

			//Returns the destination of this path (the last point in the points array)
			Geometry2d::Point::Optional destination() const;

			// Returns true if the path never touches an obstacle or additionally, when exitObstacles is true, if the path
			// starts out in an obstacle but leaves and never re-enters any obstacle.
			bool hit(const ObstacleGroup &obstacles, unsigned int start = 0) const;
			
			// Set of points in the path - used as waypoints
			std::vector<Geometry2d::Point> points;

			/**
			 * A path describes the position and velocity a robot should be at for a
			 * particular time interval.  This methd evalates the path at a given time and
			 * returns the target position and velocity of the robot.
			 *
			 * @param t Time (in seconds) since the robot started the path
			 * @param targetPosOut The position the robot would ideally be at at the given time
			 * @param targetVelOut The target velocity of the robot at the given time
			 * @return true if the path is valid at time @t, false if you've gone past the end
			 */
			//	FIXME: implement
			bool evaluate(float t, Geometry2d::Point &targetPosOut, Geometry2d::Point &targetVelOut) const;
			bool getPoint(float distance ,Geometry2d::Point &position, Geometry2d::Point &direction) const;
			void setStartSpeed(float speed);
			void setEndSpeed(float speed);
			float getStartSpeed() const;
		private:
			float startSpeed = 0;
			float endSpeed = 0;
	};
}
