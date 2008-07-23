#include "Ball.hpp"

#include <Geometry/Point2d.hpp>

using namespace Physics;

const float Ball::_radius = BALL_RADIUS;

Ball::Ball(dWorldID world, dSpaceID space) :
	Entity(world, space, 2)
{	
	_geomID = dCreateSphere (space, _radius);
	dGeomSetBody(_geomID, _bodyID);
	dGeomSetData(_geomID, (void*)Entity::Ball);
	
	//set mass parameters for ball
	//ball has a mass of ~43g
	dMassSetSphereTotal(&_mass, 0.043, _radius);
	dBodySetMass(_bodyID, &_mass);	
	
	setPosition(0, 0, .2);
	
	//dBodyAddRelForce(_bodyID, 5, 0, 0);

    _quadric = gluNewQuadric();
}

Ball::~Ball()
{
	gluDeleteQuadric(_quadric);
}

void Ball::internal()
{
	//fake friction for the ball
	const dReal* lVel = dBodyGetLinearVel(_bodyID);
	Geometry::Point2d vel(-lVel[0], -lVel[1]);
	
	if (vel.mag() != 0)
	{
		Geometry::Point2d force = vel.norm();
		force *= .03;
		dBodyAddForce(_bodyID, force.x, force.y, 0);
	}
}

void Ball::paint()
{
	const dReal* pos = dBodyGetPosition(_bodyID);
	
	glColor3ub(0xff, 0x90, 0);
	
	glPushMatrix();
		glTranslatef(pos[0], pos[1], pos[2]);
		gluSphere(_quadric, _radius, 10, 10); //radius, slices, stacks
	glPopMatrix();
}
