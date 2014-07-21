#pragma once

#include "Shape.hpp"
#include "Point.hpp"
#include "util.h"

#include <algorithm>

namespace Geometry2d
{
	class Segment;
	
	class Rect : public Shape
	{
		public:
			Rect() {}

            Rect(const Point &p1)
            {
                pt[0] = pt[1] = p1;
            }

			Rect(const Point &p1, const Point &p2)
			{
				pt[0] = p1;
				pt[1] = p2;
			}

			Rect(const Rect &other) {
				pt[0] = other.pt[0];
				pt[1] = other.pt[1];
			}

			Shape *clone() const;

			Rect &operator+=(const Point &offset)
			{
				pt[0] += offset;
				pt[1] += offset;

				return *this;
			}

			Rect &operator-=(const Point &offset)
			{
				pt[0] -= offset;
				pt[1] -= offset;

				return *this;
			}

			Rect operator+(const Point &offset)
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
			
			bool contains(const Point &other) const;
			bool contains(const Rect &other) const;

			bool containsPoint(const Point &other) const {
				return contains(other);
			}


	        bool hit(const Point &pt) const {
	        	return contains(pt);
	        }

	        bool hit(const Segment &seg) const;

			Point center() const { return (pt[0] + pt[1]) / 2; }

			void expand(const Point &pt);
			void expand(const Rect &rect);

			float minx() const { return std::min(pt[0].x, pt[1].x); }
			float miny() const { return std::min(pt[0].y, pt[1].y); }
			float maxx() const { return std::max(pt[0].x, pt[1].x); }
			float maxy() const { return std::max(pt[0].y, pt[1].y); }
			
			bool nearPoint(const Point &pt, float threshold) const;
			bool nearSegment(const Segment &seg, float threshold) const;
			
			bool intersects(const Rect &other) const;
			
			Point pt[2];

			std::string toString() {
				std::stringstream str;
				str << "Line<" << pt[0] << ", " << pt[1] << ">";
				return str.str();
			}
	};
}
