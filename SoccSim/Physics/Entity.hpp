#ifndef _ENTITY_HPP
#define _ENTITY_HPP

#include <NxPhysics.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <Geometry/Point2d.hpp>

class Entity
{
    public:
        Entity(NxScene& scene);
        virtual ~Entity();
        
        /** set position of the robot */
        virtual void position(float x, float y) = 0;
        
        /** @return the world position */
        Geometry::Point2d getPosition() const;
        
	private:
	    Entity& operator &= (Entity&);

	protected:
	    NxScene& _scene;
	    
	    /** the primary actor for the entity */
	    NxActor* _actor;
};

#endif /* _ENTITY_HPP */
