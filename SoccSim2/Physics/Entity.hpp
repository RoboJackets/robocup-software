#ifndef _ENTITY_HPP
#define _ENTITY_HPP

#include <NxPhysics.h>

#include <GL/gl.h>
#include <GL/glu.h>

class Entity
{
    public:
        Entity(NxScene& scene);
        virtual ~Entity();
        
        /** draw the entity */
        virtual void paint() const = 0;
        
	private:
	    Entity& operator &= (Entity&);
	    
	protected:
	    NxScene& _scene;
	    /** the primary actor for the entity */
	    NxActor* _actor;
};

#endif /* _ENTITY_HPP */
