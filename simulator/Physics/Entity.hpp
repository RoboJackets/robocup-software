#pragma once

#include <Geometry2d/Point.hpp>

class Env;

class Entity
{
public:
	Entity(Env *env);
	virtual ~Entity();

	/** set position of the robot */
	virtual void position(float x, float y) = 0;

	/** @return the world position */
	Geometry2d::Point getPosition() const;

private:
	Entity& operator &= (Entity&);

protected:
	Env* _env;
};
