#ifndef _XISOP_GEOMETRY_LINE_HPP
#define _XISOP_GEOMETRY_LINE_HPP

#include "Point2d.hpp"
#include "Circle2d.hpp"
#include "TransformMatrix.hpp"

namespace Geometry
{
	class Line2d
	{
		public:
			Line2d() {}
			
			Line2d(const Line2d &other)
			{
				pt[0] = other.pt[0];
				pt[1] = other.pt[1];
			}
			
			Line2d(Point2d p1, Point2d p2)
			{
				pt[0] = p1;
				pt[1] = p2;
			}

            Point2d delta() const
            {
                return pt[1] - pt[0];
            }

			/**
			returns the shortest distance between the line and a point
			@param other the point to find the distance to
			@return the distance to the point from the line
			*/
			float distTo(const Point2d &other) const;
			
			/**
			Applies a transformation matrix to the line.
			@param t the transformation matrix to perform on the line
			*/
			void transform(const TransformMatrix &t)
			{
				pt[0] = t * pt[0];
				pt[1] = t * pt[1];
			}
			
			/**
			Test for line intersections.
			If the two lines intersect, then the function returns true, else false.
			Also, if the lines intersect, then intr is set to the intersection point.
			@param other the line to test for intersection with
			@param intr set to the intersection point if the lines intersect
			@return true if the lines intersect, false otherwise
			*/
			bool intersects(const Line2d &other, Point2d *intr = 0) const;
			
			/** returns the points of intersection b/t circle and line */
			bool intersects(const Circle2d& circle, Point2d* p1 = 0, Point2d* p2 = 0) const;
			
			/**
			 * Returns the point on the line closest to p.
			 */
			Point2d nearest_point(Point2d p) const;
			
			/** the line consists of two points */
			Point2d pt[2];
	};
}

#endif
