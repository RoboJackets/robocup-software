#include "SimBall.hpp"
#include "Environment.hpp"
#include <physics/PhysicsConstants.hpp>


SimBall::SimBall(Environment* env) :
		Ball(env), _ball(0), _ballShape(0), _simEngine(env->getSimEngine())
	{
	}

void SimBall::position(float x, float y) {
	if(!_ball)
		return;
	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(btVector3(y*mtodm,Sim_Ball_Radius,x*mtodm));
	_ball->setCenterOfMassTransform(trans);
}

void SimBall::velocity(float x, float y) {
	if(_ball)
		_ball->setLinearVelocity(btVector3(y,0,x));
}

Geometry2d::Point SimBall::getPosition() const {
	return Geometry2d::Point();
}

void SimBall::initPhysics() {
	// Create ball shape
	btSphereShape* _sphereShape = new btSphereShape(Sim_Ball_Radius);
	_simEngine->addCollisionShape((btCollisionShape *) _sphereShape);

	btVector3* color = new btVector3(1.0f,0.5f,0.f);
	_sphereShape->setUserPointer(color); // rendering color

	// Create rigid body
	btTransform ballTr;
	ballTr.setIdentity();
	ballTr.setOrigin(btVector3(0,Sim_Ball_Radius+5,0));

	_ball = _simEngine->localCreateRigidBody(Sim_Ball_Mass,ballTr,_sphereShape);
	_ball->setActivationState(DISABLE_DEACTIVATION);

	resetScene();
}

void SimBall::resetScene() {
	if(_ball){
		_ball->setCenterOfMassTransform(((btDefaultMotionState* ) _ball->getMotionState())->m_startWorldTrans);
		_ball->setLinearVelocity(btVector3(0, 0, 0));
		_ball->setAngularVelocity(btVector3(0, 0, 0));
	}
}
