
#pragma once

#include "Rect.hpp"
#include "Point.hpp"
#include "Line.hpp"
#include "Circle.hpp"

namespace Geometry2d
{	
	class Rect;
	
	class Segment: public Line
	{
		public:
			Segment() {}
			
			Segment(const Segment &other): Line(other) {}
			Segment(Point p1, Point p2): Line(p1, p2) {}
			
			Segment &operator+=(const Point &delta)
			{
				pt[0] += delta;
				pt[1] += delta;
				
				return *this;
			}
			
			Point center() const
			{
				return (pt[0] + pt[1]) / 2;
			}
			
			/* returns a bounding box of type Rect */
			Rect bbox() const;
			
			/* returns the distance to point other */
			float distTo(const Point &other) const;
			
			/* returns the length of the segment */
			float length() const { return (pt[1] - pt[0]).mag(); }
			
			bool nearPoint(const Point &point, float threshold) const;
			bool nearPointPerp(const Point &point, float threshold) const;
			bool nearSegment(const Segment &other, float threshold) const;
			
			/** find the nearest point on the segment given @a p */
			Point nearestPoint(const Point& p) const;
			
			bool intersects(const Segment &other, Point *intr = 0) const;
			bool intersects(const Circle& circle) const;
	};
}
