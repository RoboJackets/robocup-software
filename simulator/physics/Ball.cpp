#include "Ball.hpp"
#include "Environment.hpp"

#include <stdio.h>

Ball::Ball(Environment* env) :
		Entity(env), _ball(0), _ballShape(0), _simEngine(env->getSimEngine())
{
}

Ball::~Ball()
{

}

void Ball::position(float x, float y) {
	if(!_ball)
		return;
	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(y*scaling,Sim_Ball_Radius,x*scaling));
	_ball->setCenterOfMassTransform(trans);
}

void Ball::velocity(float x, float y) {
	if(_ball)
		_ball->setLinearVelocity(btVector3(y,0,x)*scaling);
}

Geometry2d::Point Ball::getPosition() const {
	const btTransform& tr = _ball->getWorldTransform();
	btVector3 pos = tr.getOrigin()/scaling;
	return Geometry2d::Point(pos.z(),pos.x());
}

void Ball::initPhysics() {
	// Create ball shape
	btSphereShape* _sphereShape = new btSphereShape(Sim_Ball_Radius);
	_sphereShape->setMargin(0.004*scaling);
	_simEngine->addCollisionShape((btCollisionShape *) _sphereShape);

	btVector3* color = new btVector3(1.0f,0.5f,0.f);
	_sphereShape->setUserPointer(color); // rendering color

	// Create rigid body
	btTransform ballTr;
	ballTr.setIdentity();
	ballTr.setOrigin(btVector3(0,Sim_Ball_Radius,0));

	_ball = _simEngine->localCreateRigidBody(Sim_Ball_Mass,ballTr,_sphereShape);
	_ball->setActivationState(DISABLE_DEACTIVATION);

	_ball->setFriction(10.f); // doesn't work?
	_ball->setDamping(0.25f,0.25f); // use damping for friction

	//_ball->setCollisionFlags(_ball->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

	//enable CCD if the object moves more than 1 meter in one simulation frame
	_ball->setCcdMotionThreshold(Sim_Ball_Radius/2.f);
	_ball->setCcdSweptSphereRadius(0.2f);

	_ball->getBroadphaseHandle()->m_collisionFilterGroup |= btBroadphaseProxy::SensorTrigger;

	_ball->setRestitution(0);

	resetScene();
}

void Ball::resetScene() {
	if(_ball){
		_ball->setCenterOfMassTransform(((btDefaultMotionState* ) _ball->getMotionState())->m_startWorldTrans);
		_ball->setLinearVelocity(btVector3(0, 0, 0));
		_ball->setAngularVelocity(btVector3(0, 0, 0));
	}
}
