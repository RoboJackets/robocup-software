#include "Entity.hpp"

Entity::Entity(NxScene& scene) :
    _scene(scene), _actor(0)
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

Geometry::Point2d Entity::getPosition() const
{
    return Geometry::Point2d(_actor->getGlobalPosition().x, _actor->getGlobalPosition().y);;
}
