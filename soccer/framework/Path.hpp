// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Segment.hpp>
#include "Obstacle.hpp"

namespace Planning
{
	class Path
	{
		public:
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
			
			// Returns the index of the point in this path nearest to pt.
			int nearestIndex(const Geometry2d::Point &pt) const;
			
			/** returns the nearest segement of @a pt to the path */
			Geometry2d::Segment nearestSegment(const Geometry2d::Point &pt) const;
			
			// Returns the shortest distance from this path to the given point
			float distanceTo(const Geometry2d::Point &pt) const;
			
			// Returns true if the path never touches an obstacle or additionally, when exitObstacles is true, if the path
			// starts out in an obstacle but leaves and never re-enters any obstacle.
			bool hit(const ObstacleGroup &obstacles, unsigned int start = 0, bool exitObstacles = false) const;
			
			// Set of points in the path - used as waypoints
			std::vector<Geometry2d::Point> points;
	};
}
