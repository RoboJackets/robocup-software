#include "Entity.hpp"

using namespace Physics;

//populate the geometry class data
dGeomClass Entity::_GeomClass = { sizeof(Entity*) , Entity::_getColliderFn, Entity::_getAABB, 0, 0};

//create the geometry glass
const int Entity::ClassNum = dCreateGeomClass (&Entity::_GeomClass);

Entity::Entity(dWorldID world, dSpaceID space, bool dynamic)
{
	//create a geometry based on this class
	//_geomID = dCreateGeom(ClassNum);
	
	//if the entity will move, a body must be created
	if (dynamic)
	{
		//create a new body
		_bodyID = dBodyCreate(world);
		
		//attach the body to the geometry
		//dGeomSetBody(_geomID, _bodyID);
		
		// starting position
		//dBodySetPosition(_bodyID, 0.0, 0.0, 3);
			
		// starting speed
		//dBodySetLinearVel(_bodyID, 0.0, 0.0, 0.0);
	}
	else
	{
		_bodyID = 0;
	}
	
	//set class data to this for proper function calling
	//dGeomSetData(_geomID, this);
	
	//add the geometry to the collision space
	//dSpaceAdd(space, _geomID);
}

Entity::~Entity()
{
	//cleanup object
	if (_bodyID != 0)
	{
		dBodyDestroy(_bodyID);
	}
	
	dGeomDestroy(_geomID);
}

Geometry::Point2d Entity::pos() const
{
	if (_bodyID)
	{
		const dReal* pos = dBodyGetPosition(_bodyID);
		return Geometry::Point2d(pos[0], pos[1]);
	}
	else
	{
		return Geometry::Point2d(-100, -100);
	}
}

void Entity::setPosition(float x, float y, float z)
{
	if (_bodyID)
	{
		dBodySetPosition(_bodyID, x, y, z);
	}
}

void Entity::setVelocity(float x, float y, float z)
{
	if (_bodyID)
	{
		dBodySetLinearVel(_bodyID, x, y, z);
	}
}
