#include "Line2d.hpp"
#include "Point2d.hpp"

using namespace Geometry;

bool Line2d::intersects(const Line2d &other, Point2d *intr) const
{
	// From Mathworld:
	//	http://mathworld.wolfram.com/Line-LineIntersection.html

	float x1 = pt[0].x;
	float y1 = pt[0].y;
	float x2 = pt[1].x;
	float y2 = pt[1].y;
	float x3 = other.pt[0].x;
	float y3 = other.pt[0].y;
	float x4 = other.pt[1].x;
	float y4 = other.pt[1].y;

	float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
	if (denom == 0)
		return false;

	if (intr)
	{
		float deta = x1 * y2 - y1 * x2;
		float detb = x3 * y4 - y3 * x4;

		intr->x = (deta * (x3 - x4) - (x1 - x2) * detb) / denom;
		intr->y = (deta * (y3 - y4) - (y1 - y2) * detb) / denom;
	}

	return true;
}

float Line2d::distTo(const Point2d &other) const
{
	Point2d delta = pt[1] - pt[0];
	float top = delta.x * (pt[0].y - other.y) - (pt[0].x - other.x) * delta.y;

	return fabs(top) / delta.mag();
}

Point2d Line2d::nearest_point(Point2d p) const
{
    Point2d v_hat = delta().norm();
    return pt[0] + v_hat * v_hat.dot(p - pt[0]);
}

bool Line2d::intersects(const Circle2d& circle, Point2d* p1, Point2d* p2) const
{
	//http://mathworld.wolfram.com/Circle-LineIntersection.html
	
	const float dx = pt[1].x - pt[0].x;
	const float dy = pt[1].y - pt[0].y;
	const float dr2 = dx*dx + dy*dy;
	const float r = circle.radius();
	
	float det = pt[0].x * pt[1].y - pt[1].x * pt[0].y;
	
	float descr = r * r * dr2 - det*det;
	
	if (descr < 0)
	{
		return false;
	}
	
	float common = sqrt(descr);
	
	float x1 = det * dy; 
	float x2 = sign(dy) * dx * common;
	
	float y1 = -det * dx;
	float y2 = fabs(dy) * common;
	
	if (p1)
	{
		float x = x1 + x2;
		float y = y1 + y2;
		*p1 = Point2d(x/dr2, y/dr2);
	}
	
	if (p2)
	{
		float x = x1 - x2;
		float y = y1 - y2;
		*p2 = Point2d(x/dr2, y/dr2);
	}
	
	return true;
}
