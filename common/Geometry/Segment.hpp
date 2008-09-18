#ifndef _XISOP_GEOMETRY_SEGMENT_HPP
#define _XISOP_GEOMETRY_SEGMENT_HPP

#include "Rect.hpp"
#include "Point2d.hpp"
#include "Line2d.hpp"
#include "Circle2d.hpp"

namespace Geometry
{	
	class Rect;
	
	class Segment: public Line2d
	{
		public:
			Segment() {}

			Segment(const Segment &other): Line2d(other) {}
			Segment(Point2d p1, Point2d p2): Line2d(p1, p2) {}

			Segment &operator+=(const Point2d &delta)
			{
				pt[0] += delta;
				pt[1] += delta;

				return *this;
			}

			/* returns a bounding box of type Rect */
			Rect bbox() const;
			
			/* returns the distance to point other */
			float distTo(const Point2d &other) const;
			
			/* returns the length of the segment */
			float length() const { return (pt[1] - pt[0]).mag(); }
			
			bool nearPoint(const Point2d &point, float threshold) const;
			
			bool intersects(Segment &other, Point2d *intr) const;
			bool intersects(Segment &other) const;
			bool intersects(Circle2d& circle) const;
	};
};

#endif
