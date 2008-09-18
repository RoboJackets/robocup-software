#include "Circle2d.hpp"
#include <stdio.h>

using namespace Geometry;

Circle2d::Circle2d()
{
	this->_radius = 0;
}

Circle2d::Circle2d(float x, float y, float radius) :
	_pos(x, y), _radius(radius) {}

Circle2d::Circle2d(Geometry::Point2d pos, float radius) :
	_pos(pos), _radius(radius) {}

Circle2d::~Circle2d()
{
}

float Circle2d::radius() const
{
	return this->_radius;
}

void Circle2d::setXY(float x, float y)
{
	setX(x);
	setY(y);
}

void Circle2d::setRadius(float radius)
{
	_radius = radius;
}

#include <stdio.h>

bool Circle2d::tangentPoints(const Geometry::Point2d src, 
	Geometry::Point2d* p1, Geometry::Point2d* p2) const
{
	if (!p1 && !p2)
	{
		return false;
	}
	
	const float dist = src.distTo(_pos);
	
	if (dist < _radius)
	{
		return false;
	}
	else if (dist == _radius)
	{
		if (p1)
		{
			*p1 = src;
		}
		
		if (p2)
		{
			*p2 = src;
		}
	}
	else
	{
		//source is outside of circle
		const float theta = asin(_radius/dist);
		const float degT = theta * 180.0f / M_PI;
		
		if (p1)
		{
			Point2d final = _pos;
			final.rotate(src, degT);
			*p1 = final;
		}
		
		if (p2)
		{
			Point2d final = _pos;
			final.rotate(src, -degT);
			*p2 = final;
		}
	}
	
	return true;
}
