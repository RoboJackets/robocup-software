#pragma once

#include <Geometry2d/Point.hpp>

class Environment;

class Entity
{
public:
	Entity(Environment *env);
	virtual ~Entity();

	/** set position of the robot */
	virtual void position(float x, float y) = 0;

	/** @return the world position */
	Geometry2d::Point getPosition() const;

private:
	Entity& operator &= (Entity&);

protected:
	Environment* _env;
};
