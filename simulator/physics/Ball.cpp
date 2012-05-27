#include "Ball.hpp"
#include "Environment.hpp"

#include <Constants.hpp>

#include <stdio.h>

Ball::Ball(Environment* env) :
Entity(env)
{
}

Ball::~Ball()
{

}

void Ball::position(float x, float y)
{
	_pos = Geometry2d::Point(x,y);
}

void Ball::velocity(float x, float y)
{
	_vel = Geometry2d::Point(x,y);
}

Geometry2d::Point Ball::getPosition() const
{
	return _pos;
}
