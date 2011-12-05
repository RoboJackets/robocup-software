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

#include "SimpleApplication.hpp"
#include "SimpleCamera.hpp"

#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"//picking
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"//picking
#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btSerializer.h"

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

////////////////////////////////////
// SimpleApplication class
////////////////////////////////////

SimpleApplication::SimpleApplication() :
		m_dynamicsWorld(0), m_shootBoxShape(0), m_debugMode(0), m_modifierKeys(0),
		m_ShootBoxInitialSpeed(40.f), m_stepping(true), m_singleStep(false),
		m_defaultContactProcessingThreshold(BT_LARGE_FLOAT), _camera(0)
{
}

SimpleApplication::~SimpleApplication() {
	if (m_shootBoxShape)
		delete m_shootBoxShape;
	if (_camera)
		delete _camera;
}

void SimpleApplication::keyboardCallback(unsigned char key, int x, int y) {
	(void) x;
	(void) y;

	m_lastKey = 0;

	switch (key) {
	case 'q':
#ifdef BT_USE_FREEGLUT
		//return from glutMainLoop(), detect memory leaks etc.
		glutLeaveMainLoop();
#else
		exit(0);
#endif
		break;
	case 'h':
		if (m_debugMode & btIDebugDraw::DBG_NoHelpText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoHelpText);
		else
			m_debugMode |= btIDebugDraw::DBG_NoHelpText;
		break;

	case 'w':
		if (m_debugMode & btIDebugDraw::DBG_DrawWireframe)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawWireframe);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
		break;

	case 'p':
		if (m_debugMode & btIDebugDraw::DBG_ProfileTimings)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_ProfileTimings);
		else
			m_debugMode |= btIDebugDraw::DBG_ProfileTimings;
		break;

	case '=': {
		int maxSerializeBufferSize = 1024 * 1024 * 5;
		btDefaultSerializer* serializer = new btDefaultSerializer(
				maxSerializeBufferSize);
		//serializer->setSerializationFlags(BT_SERIALIZE_NO_DUPLICATE_ASSERT);
		m_dynamicsWorld->serialize(serializer);
		FILE* f2 = fopen("testFile.bullet", "wb");
		fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(),
				1, f2);
		fclose(f2);
		delete serializer;
		break;

	}

	case 'm':
		if (m_debugMode & btIDebugDraw::DBG_EnableSatComparison)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableSatComparison);
		else
			m_debugMode |= btIDebugDraw::DBG_EnableSatComparison;
		break;

	case 'n':
		if (m_debugMode & btIDebugDraw::DBG_DisableBulletLCP)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DisableBulletLCP);
		else
			m_debugMode |= btIDebugDraw::DBG_DisableBulletLCP;
		break;

	case 't':
		if (m_debugMode & btIDebugDraw::DBG_DrawText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawText);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawText;
		break;
	case 'y':
		if (m_debugMode & btIDebugDraw::DBG_DrawFeaturesText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawFeaturesText);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawFeaturesText;
		break;
	case 'a':
		if (m_debugMode & btIDebugDraw::DBG_DrawAabb)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawAabb);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawAabb;
		break;
	case 'c':
		if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawContactPoints);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawContactPoints;
		break;
	case 'C':
		if (m_debugMode & btIDebugDraw::DBG_DrawConstraints)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawConstraints);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawConstraints;
		break;
	case 'L':
		if (m_debugMode & btIDebugDraw::DBG_DrawConstraintLimits)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawConstraintLimits);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawConstraintLimits;
		break;

	case 'd':
		if (m_debugMode & btIDebugDraw::DBG_NoDeactivation)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoDeactivation);
		else
			m_debugMode |= btIDebugDraw::DBG_NoDeactivation;
		if (m_debugMode & btIDebugDraw::DBG_NoDeactivation) {
			gDisableDeactivation = true;
		} else {
			gDisableDeactivation = false;
		}
		break;

	case 's':
		clientMoveAndDisplay();
		break;
		//    case ' ' : newRandom(); break;
	case ' ':
		clientResetScene();
		break;
	case '1': {
		if (m_debugMode & btIDebugDraw::DBG_EnableCCD)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableCCD);
		else
			m_debugMode |= btIDebugDraw::DBG_EnableCCD;
		break;
	}

	case '.': {
		shootBox(_camera->getRayTo(x, y)); //getCameraTargetPosition());
		break;
	}

	case '+': {
		m_ShootBoxInitialSpeed += 10.f;
		break;
	}
	case '-': {
		m_ShootBoxInitialSpeed -= 10.f;
		break;
	}

	default:
		//        std::cout << "unused key : " << key << std::endl;
		break;
	}

	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(m_debugMode);

}

void SimpleApplication::setDebugMode(int mode) {
	m_debugMode = mode;
	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(mode);
}

void SimpleApplication::moveAndDisplay() {
	clientMoveAndDisplay();
}

void SimpleApplication::setShootBoxShape() {
	if (!m_shootBoxShape) {
		btBoxShape* box = new btBoxShape(btVector3(.5f, .5f, .5f));
		box->initializePolyhedralFeatures();
		m_shootBoxShape = box;
	}
}

void SimpleApplication::shootBox(const btVector3& destination) {
	if (m_dynamicsWorld) {
		float mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = _camera->getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape();

		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,
				m_shootBoxShape);
		body->setLinearFactor(btVector3(1, 1, 1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0] - camPos[0], destination[1] - camPos[1],
				destination[2] - camPos[2]);
		linVel.normalize();
		linVel *= m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0, 0, 0, 1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0, 0, 0));
		body->setCcdMotionThreshold(0.5);
		body->setCcdSweptSphereRadius(0.9f);
	}
}

btRigidBody* SimpleApplication::localCreateRigidBody(float mass,
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

void SimpleApplication::clientResetScene() {
	int numObjects = 0;
	int i;

	if (m_dynamicsWorld) {
		int numConstraints = m_dynamicsWorld->getNumConstraints();
		for (i = 0; i < numConstraints; i++) {
			m_dynamicsWorld->getConstraint(0)->setEnabled(true);
		}
		numObjects = m_dynamicsWorld->getNumCollisionObjects();

		///create a copy of the array, not a reference!
		btCollisionObjectArray copyArray =
				m_dynamicsWorld->getCollisionObjectArray();

		for (i = 0; i < numObjects; i++) {
			btCollisionObject* colObj = copyArray[i];
			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body) {
				if (body->getMotionState()) {
					btDefaultMotionState* myMotionState =
							(btDefaultMotionState*) body->getMotionState();
					myMotionState->m_graphicsWorldTrans =
							myMotionState->m_startWorldTrans;
					body->setCenterOfMassTransform(myMotionState->m_graphicsWorldTrans);
					colObj->setInterpolationWorldTransform(
							myMotionState->m_startWorldTrans);
					colObj->forceActivationState(ACTIVE_TAG);
					colObj->activate();
					colObj->setDeactivationTime(0);
					//colObj->setActivationState(WANTS_DEACTIVATION);
				}
				//removed cached contact points (this is not necessary if all objects have been removed from the dynamics world)
				if (m_dynamicsWorld->getBroadphase()->getOverlappingPairCache())
					m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
							colObj->getBroadphaseHandle(),
							getDynamicsWorld()->getDispatcher());

				btRigidBody* body = btRigidBody::upcast(colObj);
				if (body && !body->isStaticObject()) {
					btRigidBody::upcast(colObj)->setLinearVelocity(btVector3(0, 0, 0));
					btRigidBody::upcast(colObj)->setAngularVelocity(btVector3(0, 0, 0));
				}
			}
		}

		///reset some internal cached data in the broadphase
		m_dynamicsWorld->getBroadphase()->resetPool(
				getDynamicsWorld()->getDispatcher());
		m_dynamicsWorld->getConstraintSolver()->reset();
	}
}

void SimpleApplication::updateModifierKeys() {
	m_modifierKeys = 0;
	if (glutGetModifiers() & GLUT_ACTIVE_ALT)
		m_modifierKeys |= BT_ACTIVE_ALT;

	if (glutGetModifiers() & GLUT_ACTIVE_CTRL)
		m_modifierKeys |= BT_ACTIVE_CTRL;

	if (glutGetModifiers() & GLUT_ACTIVE_SHIFT)
		m_modifierKeys |= BT_ACTIVE_SHIFT;
}

void SimpleApplication::specialKeyboard(int key, int x, int y) {
	(void) x;
	(void) y;

	switch (key) {
	case GLUT_KEY_END: {
		int numObj = getDynamicsWorld()->getNumCollisionObjects();
		if (numObj) {
			btCollisionObject* obj =
					getDynamicsWorld()->getCollisionObjectArray()[numObj - 1];

			getDynamicsWorld()->removeCollisionObject(obj);
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState()) {
				delete body->getMotionState();
			}
			delete obj;
		}
		break;
	}
	default:
		//        std::cout << "unused (special) key : " << key << std::endl;
		break;
	}
	glutPostRedisplay();
}

