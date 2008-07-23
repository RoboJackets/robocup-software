#ifndef ENTITY_HPP_
#define ENTITY_HPP_

#include <ode/ode.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include <Geometry/Point2d.hpp>
#include <Sizes.h>

namespace Physics
{
	class Entity
	{
		public:
			typedef enum
			{
				RobotShell,
				Floor,
				Wall,
				Ball,
				Roller,
				Kicker
			} Geom;
		
		public:
			Entity(dWorldID world, dSpaceID space, bool dynamic = true);
			virtual ~Entity();
			
			//int object() const { return _obj; }
			
			virtual int collide(dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip)
			{
				return 0;
			}
			
			virtual void getAABB (dGeomID g, dReal aabb[6])
			{
				memset(aabb, 0, sizeof(dReal)*6);;
			}
			
			/** entities overload this to be drawn */
			virtual void paint() = 0;
			
			/** ODE geometry class num, returned from dGeomGetClass(dGeomID) */
			static const int ClassNum;
			
			/** return the position of the entity */
			Geometry::Point2d pos() const;
			
			/** set the position of the entity */
			virtual void setPosition(float x=0, float y=0, float z=0);
			
			virtual void setVelocity(float x=0, float y=0, float z=0);
			
			/** entities that wish to perform tasks before collision detection
			 *  I.E. robots wanting to kick and such */
			virtual void internal() {};
            
            dBodyID body() const { return _bodyID; }
			
		/// ODE wrappers ///
		private:
			static int _collide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip)
			{
				return ((Entity*)dGeomGetData(o1))->collide(o1, o2, flags, contact, skip);
			}
			
			static dColliderFn* _getColliderFn (int num) { return _collide; }
			
			static void _getAABB (dGeomID g, dReal aabb[6])
			{
				((Entity*)dGeomGetData(g))->getAABB(g, aabb);
			}
			
		protected:
			//const Object _obj;
			
			dBodyID _bodyID;
			dGeomID _geomID;
			dMass _mass;
			
		private:
			static dGeomClass _GeomClass;
	};
}

#endif /*ENTITY_HPP_*/
