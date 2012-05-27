#pragma once

#include "Entity.hpp"


class Ball: public Entity
{
private:
	// temp state info
	Geometry2d::Point _pos, _vel;

public:
	Ball(Environment* env);
	virtual ~Ball();

	virtual void position(float x, float y);
	virtual void velocity(float x, float y);

	virtual Geometry2d::Point getPosition() const;
};
