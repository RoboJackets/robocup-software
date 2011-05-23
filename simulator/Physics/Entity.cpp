#include "Entity.hpp"
#include "Env.hpp"

Entity::Entity(Env* env) :
    _env(env)
{
    
}

Entity::~Entity()
{
}

Geometry2d::Point Entity::getPosition() const
{
	Geometry2d::Point result;
	return result;
}
