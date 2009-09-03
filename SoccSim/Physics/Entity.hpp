#pragma once

#include <NxPhysics.h>

#include <GL/gl.h>
#include <GL/glu.h>

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
        
        NxActor *actor() const
        {
            return _actor;
        }
        
	private:
	    Entity& operator &= (Entity&);

	protected:
        Env* _env;
	    NxScene& _scene;
	    
	    /** the primary actor for the entity */
	    NxActor* _actor;
};
