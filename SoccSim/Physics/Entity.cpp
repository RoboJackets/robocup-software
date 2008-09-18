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
