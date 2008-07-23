#include "Point2d.hpp"
#include "util.h"

#include <math.h>

using namespace Geometry;

void Point2d::rotate(const Point2d &origin, float angle)
{
	float ox = x - origin.x;
	float oy = y - origin.y;

	float newX = ox * cos(angle * DEG_TO_RAD) - oy * sin(angle * DEG_TO_RAD);
	float newY = oy * cos(angle * DEG_TO_RAD) + ox * sin(angle * DEG_TO_RAD);

	x = newX + origin.x;
	y = newY + origin.y;
}

bool Point2d::inThreshold(const Point2d &other, float threshold) const
{
	Point2d delta = *this - other;
	return delta.magsq() <= (threshold * threshold);
}
