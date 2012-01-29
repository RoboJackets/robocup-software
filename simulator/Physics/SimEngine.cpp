/*
 Bullet Continuous Collision Detection and Physics Library
 Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:

 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 */

#include <boost/tuple/tuple.hpp>

/// September 2006: VehicleDemo is work in progress, this file is mostly just a placeholder
/// This VehicleDemo file is very early in development, please check it later
/// One todo is a basic engine model:
/// A function that maps user input (throttle) into torque/force applied on the wheels
/// with gears etc.

#include "GlutCamera.hpp"
#include "VehicleDemo.hpp"

#include <btBulletDynamicsCommon.h>

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging
#include "GL_ShapeDrawer.h"

const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts

#define CUBE_HALF_EXTENTS 1

using namespace std;

////////////////////////////////////
// SimEngine class
////////////////////////////////////

void SimEngine::initPhysics() {
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,
			m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);
}

SimEngine::~SimEngine() {
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	for (int i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--) {
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState()) {
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++) {
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void SimEngine::setDebug(const btIDebugDraw::DebugDrawModes& flag) {
	if (m_debugMode & flag)
		m_debugMode = m_debugMode & (~flag);
	else
		m_debugMode |= flag;
}

btRigidBody* SimEngine::localCreateRigidBody(float mass,
		const btTransform& startTransform, btCollisionShape* shape) {
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(
			startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape,
			localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);
#endif//
	m_dynamicsWorld->addRigidBody(body);

	return body;
}

btScalar SimEngine::getDeltaTimeMicroseconds() {
	btScalar dt = (btScalar) m_clock.getTimeMicroseconds();
	m_clock.reset();
	return dt;
}

void SimEngine::stepSimulation() {
	float dt = getDeltaTimeMicroseconds() * 0.000001f;
	if (m_dynamicsWorld) {
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = 2;
		m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);
	}
}

void SimEngine::debugDrawWorld() {
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
}



