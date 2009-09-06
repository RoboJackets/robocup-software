#include "Point.hpp"
#include "util.h"

#include <math.h>

using namespace Geometry2d;

void Point::rotate(const Point &origin, float angle)
{
	float ox = x - origin.x;
	float oy = y - origin.y;

	float newX = ox * cos(angle * DegreesToRadians) - oy * sin(angle * DegreesToRadians);
	float newY = oy * cos(angle * DegreesToRadians) + ox * sin(angle * DegreesToRadians);

	x = newX + origin.x;
	y = newY + origin.y;
}

float Point::angleTo(const Point& other) const
{
	return acos(this->normalized().dot(other.normalized()));
}
