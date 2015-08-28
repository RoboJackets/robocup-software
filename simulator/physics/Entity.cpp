#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>

#include "Entity.hpp"
#include "Environment.hpp"

Entity::Entity(Environment* env) : _env(env) {}

Entity::~Entity() {}

Geometry2d::Point Entity::getPosition() const {
    Geometry2d::Point result;
    // placeholder
    return result;
}
