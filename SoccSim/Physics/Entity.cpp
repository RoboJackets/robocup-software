#include "Entity.hpp"
#include "Env.hpp"

Entity::Entity(Env* env) :
    _env(env),
    _scene(*(env->scene())),
    _actor(0)
{
    
}

Entity::~Entity()
{
    if (_actor)
    {
        _scene.releaseActor(*_actor);
        _actor = 0;
    }
}

Geometry2d::Point Entity::getPosition() const
{
    return Geometry2d::Point(_actor->getGlobalPosition().x, _actor->getGlobalPosition().y);;
}
