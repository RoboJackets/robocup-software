#ifndef _XISOP_GEOMETRY_RECT_HPP
#define _XISOP_GEOMETRY_RECT_HPP

#include "Point2d.hpp"
#include "util.h"

#include <algorithm>

namespace Geometry
{
	class Segment;
	
	class Rect
	{
		public:
			Rect() {}

			Rect(const Point2d &p1, const Point2d &p2)
			{
				pt[0] = p1;
				pt[1] = p2;
			}

			Rect &operator+=(const Point2d &offset)
			{
				pt[0] += offset;
				pt[1] += offset;

				return *this;
			}

			Rect &operator-=(const Point2d &offset)
			{
				pt[0] -= offset;
				pt[1] -= offset;

				return *this;
			}

			Rect operator+(const Point2d &offset)
			{
				return Rect(pt[0] + offset, pt[1] + offset);
			}

			Rect operator*(float s)
			{
				return Rect(pt[0] * s, pt[1] * s);
			}

			Rect &operator*=(float s)
			{
				pt[0] *= s;
				pt[1] *= s;

				return *this;
			}
			
			bool contains(const Point2d &other) const;
			bool contains(const Rect &other) const;

			Point2d center() const { return (pt[0] + pt[1]) / 2; }

			void expand(const Point2d &pt);
			void expand(const Rect &rect);

			float minx() const { return std::min(pt[0].x, pt[1].x); }
			float miny() const { return std::min(pt[0].y, pt[1].y); }
			float maxx() const { return std::max(pt[0].x, pt[1].x); }
			float maxy() const { return std::max(pt[0].y, pt[1].y); }
			
			bool nearPoint(const Point2d &pt, float threshold) const;
			bool nearSegment(const Segment &seg, float threshold) const;
			
			bool intersects(const Rect &other) const;
			
			Point2d pt[2];
	};
};

#endif
