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
#include <btBulletDynamicsCommon.h>

#include "SimpleCamera.hpp"

#include "BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"

#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btSerializer.h"

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"//picking
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"//picking

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging
#include "GL_ShapeDrawer.h"

#include "VehicleDemo.hpp"

const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts

#define CUBE_HALF_EXTENTS 1

using namespace std;

////////////////////////////////////
// Vehicle class
////////////////////////////////////
// static physics parameters
static const float maxEngineForce = 1000.f; //this should be engine/velocity dependent
static const float maxBreakingForce = 100.f;
static const float steeringIncrement = 0.04f;
static const float steeringClamp = 0.3f;
static const float wheelRadius = 0.5f;
static const float wheelWidth = 0.4f;
static const float wheelFriction = 1000; //BT_LARGE_FLOAT;
static const float suspensionStiffness = 20.f;
static const float suspensionDamping = 2.3f;
static const float suspensionCompression = 4.4f;
static const float rollInfluence = 0.1f; //1.0f;
static const btScalar suspensionRestLength = 0.6f;

void Vehicle::initPhysics(VehicleDemo* env) {

	// Assumes that Y is up
	int rightIndex = 0;
	int upIndex = 1;
	int forwardIndex = 2;
	btVector3 wheelDirectionCS0(0, -1, 0);
	btVector3 wheelAxleCS(-1, 0, 0);

	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
	m_collisionShapes->push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes->push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 1, 0));

	compound->addChildShape(localTrans, chassisShape);

	btTransform vehicleTr;
	vehicleTr.setIdentity();
	vehicleTr.setOrigin(btVector3(0, 0.f, 0));

	m_carChassis = env->localCreateRigidBody(800, vehicleTr, compound);
	//m_carChassis->setDamping(0.2,0.2);

	m_wheelShape = new btCylinderShapeX(
			btVector3(wheelWidth, wheelRadius, wheelRadius));

	resetScene();

	/// create vehicle
	{

		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis,
				m_vehicleRayCaster);

		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addVehicle(m_vehicle);

		float connectionHeight = 1.2f;

		bool isFrontWheel = true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS - (0.3 * wheelWidth),
				connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);

		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheelWidth),
				connectionHeight, 2 * CUBE_HALF_EXTENTS - wheelRadius);

		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS + (0.3 * wheelWidth),
				connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);

		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS - (0.3 * wheelWidth),
				connectionHeight, -2 * CUBE_HALF_EXTENTS + wheelRadius);
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

		for (int i = 0; i < m_vehicle->getNumWheels(); i++) {
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}
}

void Vehicle::steerLeft() {
	gVehicleSteering += steeringIncrement;
	if (gVehicleSteering > steeringClamp)
		gVehicleSteering = steeringClamp;
}

void Vehicle::steerRight() {
	gVehicleSteering -= steeringIncrement;
	if (gVehicleSteering < -steeringClamp)
		gVehicleSteering = -steeringClamp;
}

void Vehicle::driveForward() {
	gEngineForce = maxEngineForce;
	gBreakingForce = 0.f;
}

void Vehicle::driveBackward() {
	gBreakingForce = maxBreakingForce;
	gEngineForce = 0.f;
}

void Vehicle::move() {
	int wheelIndex = 2;
	m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
	m_vehicle->setBrake(gBreakingForce, wheelIndex);
	wheelIndex = 3;
	m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
	m_vehicle->setBrake(gBreakingForce, wheelIndex);

	wheelIndex = 0;
	m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
	wheelIndex = 1;
	m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
}

void Vehicle::drawWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax) {
	btVector3 wheelColor(1, 0, 0);
	btScalar m[16];
	for (int i = 0; i < m_vehicle->getNumWheels(); i++) {
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		int debug_mode = m_dynamicsWorld->getDebugDrawer()->getDebugMode();
		shapeDrawer->drawOpenGL(m, m_wheelShape, wheelColor, debug_mode,
				worldBoundsMin, worldBoundsMax);
	}
}

void Vehicle::resetScene() {
	gVehicleSteering = 0.f;
	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
	m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
			m_carChassis->getBroadphaseHandle(), m_dynamicsWorld->getDispatcher());
	if (m_vehicle) {
		m_vehicle->resetSuspension();
		for (int i = 0; i < m_vehicle->getNumWheels(); i++) {
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i, true);
		}
	}
}

void Vehicle::getWorldTransform(btTransform& chassisWorldTrans) const {
	m_carChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
}

////////////////////////////////////
// GroundSurface class
////////////////////////////////////

void GroundSurface::initPhysics(VehicleDemo* env)
{
	btCollisionShape* groundShape = new btBoxShape(btVector3(50, 3, 50));
	m_collisionShapes->push_back(groundShape);
	btTransform tr;
	tr.setIdentity();

	// use triangle mesh for ground
	int i;

	const float TRIANGLE_SIZE = 20.f;

	//create a triangle-mesh ground
	int vertStride = sizeof(btVector3);
	int indexStride = 3 * sizeof(int);

	const int NUM_VERTS_X = 20;
	const int NUM_VERTS_Y = 20;
	const int totalVerts = NUM_VERTS_X * NUM_VERTS_Y;

	const int totalTriangles = 2 * (NUM_VERTS_X - 1) * (NUM_VERTS_Y - 1);

	m_vertices = new btVector3[totalVerts];
	int* gIndices = new int[totalTriangles * 3];

	for (i = 0; i < NUM_VERTS_X; i++) {
		for (int j = 0; j < NUM_VERTS_Y; j++) {
			float height = 0.f;
			m_vertices[i + j * NUM_VERTS_X].setValue(
					(i - NUM_VERTS_X * 0.5f) * TRIANGLE_SIZE, height,
					(j - NUM_VERTS_Y * 0.5f) * TRIANGLE_SIZE);
		}
	}

	int index = 0;
	for (i = 0; i < NUM_VERTS_X - 1; i++) {
		for (int j = 0; j < NUM_VERTS_Y - 1; j++) {
			gIndices[index++] = j * NUM_VERTS_X + i;
			gIndices[index++] = j * NUM_VERTS_X + i + 1;
			gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;

			gIndices[index++] = j * NUM_VERTS_X + i;
			gIndices[index++] = (j + 1) * NUM_VERTS_X + i + 1;
			gIndices[index++] = (j + 1) * NUM_VERTS_X + i;
		}
	}

	m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles, gIndices,
			indexStride, totalVerts, (btScalar*) &m_vertices[0].x(), vertStride);

	bool useQuantizedAabbCompression = true;
	groundShape = new btBvhTriangleMeshShape(m_indexVertexArrays,
			useQuantizedAabbCompression);

	tr.setOrigin(btVector3(0, -4.5f, 0));

	m_collisionShapes->push_back(groundShape);

	//create ground object
	env->localCreateRigidBody(0, tr, groundShape);
}

GroundSurface::GroundSurface(btDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* collision_shapes)
: m_indexVertexArrays(0), m_vertices(0), m_dynamicsWorld(world), m_collisionShapes(collision_shapes)
{
}

GroundSurface::~GroundSurface() {
	delete m_indexVertexArrays;
	delete m_vertices;
}

////////////////////////////////////
// SimpleApplication class
////////////////////////////////////

SimpleApplication::SimpleApplication() :
		m_dynamicsWorld(0),m_stepping(true), m_singleStep(false),
		m_debugMode(0),
		m_defaultContactProcessingThreshold(BT_LARGE_FLOAT),
		_camera(0)
{
}

SimpleApplication::~SimpleApplication() {
	if (_camera)
		delete _camera;
}

void SimpleApplication::keyboardCallback(unsigned char key, int x, int y) {
	(void) x;
	(void) y;

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

////////////////////////////////////
// VehicleDemo class
////////////////////////////////////
VehicleDemo::VehicleDemo() :
	_vehicle(0), _ground(0), m_cameraHeight(4.f),
	m_minCameraDistance(3.f), m_maxCameraDistance(10.f)
{
}

VehicleDemo::~VehicleDemo() {
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

	delete _ground;

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete vehicle
	delete _vehicle;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

void VehicleDemo::initPhysics() {

	// set up the simulation
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin, worldMax);
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,
			m_overlappingPairCache, m_constraintSolver, m_collisionConfiguration);

	// Set up the ground
	_ground = new GroundSurface(m_dynamicsWorld, &m_collisionShapes);
	_ground->initPhysics(this);

	// Set up the vehicle
	_vehicle = new Vehicle(m_dynamicsWorld, &m_collisionShapes);
	_vehicle->initPhysics(this);

	// Set up the camera
	_camera = new GlutCamera;
	_camera->setCameraPosition(btVector3(30, 30, 30));
	_camera->setCameraDistance(26.f);
	_camera->setDynamicsWorld(m_dynamicsWorld);
}

//to be implemented by the demo
void VehicleDemo::renderme() {
	updateCamera();

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,
			worldBoundsMax);

	_vehicle->drawWheels(camera()->shapeDrawer(), worldBoundsMin, worldBoundsMax);

	_camera->renderme(m_debugMode);
}

void VehicleDemo::clientMoveAndDisplay() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	_vehicle->move();

	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	if (m_dynamicsWorld) {
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = 2;
		m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);
	}

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void VehicleDemo::displayCallback(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void VehicleDemo::clientResetScene() {
	if (_vehicle)
		_vehicle->resetScene();
}

void VehicleDemo::specialKeyboardUp(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_UP: {
		_vehicle->gEngineForce = 0.f;
		break;
	}
	case GLUT_KEY_DOWN: {
		_vehicle->gBreakingForce = 0.f;
		break;
	}
	default:
		SimpleApplication::specialKeyboardUp(key, x, y);
		break;
	}
}

void VehicleDemo::specialKeyboard(int key, int x, int y) {
	//	printf("key = %i x=%i y=%i\n",key,x,y);
	switch (key) {
	case GLUT_KEY_LEFT: {
		_vehicle->steerLeft();
		break;
	}
	case GLUT_KEY_RIGHT: {
		_vehicle->steerRight();
		break;
	}
	case GLUT_KEY_UP: {
		_vehicle->driveForward();
		break;
	}
	case GLUT_KEY_DOWN: {
		_vehicle->driveBackward();
		break;
	}
	default:
		SimpleApplication::specialKeyboard(key, x, y);
		break;
	}
}

void VehicleDemo::updateCamera() {
	if (!_vehicle || !_camera) return;

	// calculate where the camera should be
	btTransform chassisWorldTrans;
	_vehicle->getWorldTransform(chassisWorldTrans);

	btVector3 targetPosition = chassisWorldTrans.getOrigin();
	btVector3 cameraPosition = camera()->getCameraPosition();

	//interpolate the camera height
	cameraPosition[1] = (15.0 * cameraPosition[1] + targetPosition[1] + m_cameraHeight) / 16.0;

	btVector3 camToObject = targetPosition - cameraPosition;

	//keep distance between min and max distance
	float cameraDistance = camToObject.length();
	float correctionFactor = 0.f;
	if (cameraDistance < m_minCameraDistance) {
		correctionFactor = 0.15 * (m_minCameraDistance - cameraDistance)
						/ cameraDistance;
	}
	if (cameraDistance > m_maxCameraDistance) {
		correctionFactor = 0.15 * (m_maxCameraDistance - cameraDistance)
						/ cameraDistance;
	}
	cameraPosition -= correctionFactor * camToObject;

	_camera->setCameraPosition(cameraPosition);
	_camera->setCameraTargetPosition(targetPosition);

	// rendering details
	_camera->updateCamera(); // default camera
//	_camera->chaseCamera(); // based on the original version
}

