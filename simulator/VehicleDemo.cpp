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

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging
#include "GL_ShapeDrawer.h"

#include <GlutStuff.h>
#include "VehicleDemo.h"

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
// VehicleDemo class
////////////////////////////////////
pair<btCollisionShape*, btTransform> VehicleDemo::addGround()
{
	btCollisionShape* groundShape = new btBoxShape(btVector3(50, 3, 50));
	m_collisionShapes.push_back(groundShape);
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

	m_collisionShapes.push_back(groundShape);

	//create ground object
	localCreateRigidBody(0, tr, groundShape);

	return make_pair(groundShape, tr);
}

////////////////////////////////////
VehicleDemo::VehicleDemo() :
	_vehicle(0), m_indexVertexArrays(0), m_vertices(0), m_cameraHeight(4.f),
	m_minCameraDistance(3.f), m_maxCameraDistance(10.f) {
	m_cameraPosition = btVector3(30, 30, 30);
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

	delete m_indexVertexArrays;
	delete m_vertices;

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
	btCollisionShape* groundShape;
	btTransform groundTr;
	boost::tie(groundShape, groundTr) = addGround();

	//create ground object - static if mass is zero
	localCreateRigidBody(0, groundTr, groundShape);

	// Set up the vehicle
	_vehicle = new Vehicle(m_dynamicsWorld, &m_collisionShapes);
	_vehicle->initPhysics(this);

	setCameraDistance(26.f);
}

//to be implemented by the demo
void VehicleDemo::renderme() {
	updateCamera();

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,
			worldBoundsMax);

	_vehicle->drawWheels(m_shapeDrawer, worldBoundsMin, worldBoundsMax);
	DemoApplication::renderme();
}

void VehicleDemo::clientMoveAndDisplay() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	_vehicle->move();
//	{
//		int wheelIndex = 2;
//		m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
//		m_vehicle->setBrake(gBreakingForce, wheelIndex);
//		wheelIndex = 3;
//		m_vehicle->applyEngineForce(gEngineForce, wheelIndex);
//		m_vehicle->setBrake(gBreakingForce, wheelIndex);
//
//		wheelIndex = 0;
//		m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
//		wheelIndex = 1;
//		m_vehicle->setSteeringValue(gVehicleSteering, wheelIndex);
//	}

	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	if (m_dynamicsWorld) {
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 2;
		if (m_idle)
			dt = 1.0 / 420.f;

//		int numSimSteps =
			m_dynamicsWorld->stepSimulation(dt, maxSimSubSteps);

		//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_FEEDBACK
	}

#ifdef USE_QUICKPROF 
	btProfiler::beginBlock("render");
#endif //USE_QUICKPROF 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

#ifdef USE_QUICKPROF 
	btProfiler::endBlock("render");
#endif 

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
//	gVehicleSteering = 0.f;
//	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
//	m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
//	m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
//	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
//			m_carChassis->getBroadphaseHandle(), getDynamicsWorld()->getDispatcher());
//	if (m_vehicle) {
//		m_vehicle->resetSuspension();
//		for (int i = 0; i < m_vehicle->getNumWheels(); i++) {
//			//synchronize the wheels with the (interpolated) chassis worldtransform
//			m_vehicle->updateWheelTransform(i, true);
//		}
//	}
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
		DemoApplication::specialKeyboardUp(key, x, y);
		break;
	}
}

void VehicleDemo::specialKeyboard(int key, int x, int y) {
	//	printf("key = %i x=%i y=%i\n",key,x,y);
	switch (key) {
	case GLUT_KEY_LEFT: {
		_vehicle->steerLeft();
//		gVehicleSteering += steeringIncrement;
//		if (gVehicleSteering > steeringClamp)
//			gVehicleSteering = steeringClamp;

		break;
	}
	case GLUT_KEY_RIGHT: {
		_vehicle->steerRight();
//		gVehicleSteering -= steeringIncrement;
//		if (gVehicleSteering < -steeringClamp)
//			gVehicleSteering = -steeringClamp;

		break;
	}
	case GLUT_KEY_UP: {
		_vehicle->driveForward();
//		gEngineForce = maxEngineForce;
//		gBreakingForce = 0.f;
		break;
	}
	case GLUT_KEY_DOWN: {
		_vehicle->driveBackward();
//		gBreakingForce = maxBreakingForce;
//		gEngineForce = 0.f;
		break;
	}
	default:
		DemoApplication::specialKeyboard(key, x, y);
		break;
	}
}

void VehicleDemo::updateCamera() {

	//#define DISABLE_CAMERA 1
#ifdef DISABLE_CAMERA
	DemoApplication::updateCamera();
	return;
#endif //DISABLE_CAMERA
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	btTransform chassisWorldTrans;
	_vehicle->getWorldTransform(chassisWorldTrans);
//	m_carChassis->getMotionState()->getWorldTransform(chassisWorldTrans);

	m_cameraTargetPosition = chassisWorldTrans.getOrigin();

	//interpolate the camera height
	m_cameraPosition[1] = (15.0 * m_cameraPosition[1] + m_cameraTargetPosition[1]
	                                                                           + m_cameraHeight) / 16.0;

	btVector3 camToObject = m_cameraTargetPosition - m_cameraPosition;

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
	m_cameraPosition -= correctionFactor * camToObject;

	btScalar aspect = m_glutScreenWidth / (btScalar) m_glutScreenHeight;
	glFrustum(-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2],
			m_cameraTargetPosition[0], m_cameraTargetPosition[1],
			m_cameraTargetPosition[2], m_cameraUp.getX(), m_cameraUp.getY(),
			m_cameraUp.getZ());

}

