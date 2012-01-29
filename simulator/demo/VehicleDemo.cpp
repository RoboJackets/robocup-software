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

void Vehicle::initPhysics() {

	// Assumes that Y is up
	int rightIndex = 0;
	int upIndex = 1;
	int forwardIndex = 2;
	btVector3 wheelDirectionCS0(0, -1, 0);
	btVector3 wheelAxleCS(-1, 0, 0);

	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
	_simEngine->addCollisionShape(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	_simEngine->addCollisionShape(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0, 1, 0));

	compound->addChildShape(localTrans, chassisShape);

	btTransform vehicleTr;
	vehicleTr.setIdentity();
	vehicleTr.setOrigin(btVector3(0, 0.f, 0));

	m_carChassis = _simEngine->localCreateRigidBody(800, vehicleTr, compound);
	//m_carChassis->setDamping(0.2,0.2);

	m_wheelShape = new btCylinderShapeX(
			btVector3(wheelWidth, wheelRadius, wheelRadius));

	resetScene();

	/// create vehicle
	{

		m_vehicleRayCaster = new btDefaultVehicleRaycaster(_simEngine->dynamicsWorld());
		m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);

		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		_simEngine->addVehicle(m_vehicle);

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
		int debug_mode = _simEngine->dynamicsWorld()->getDebugDrawer()->getDebugMode();
		shapeDrawer->drawOpenGL(m, m_wheelShape, wheelColor, debug_mode,
				worldBoundsMin, worldBoundsMax);
	}
}

void Vehicle::resetScene() {
	gVehicleSteering = 0.f;
	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
	m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
	_simEngine->dynamicsWorld()->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
			m_carChassis->getBroadphaseHandle(), _simEngine->dynamicsWorld()->getDispatcher());
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

void GroundSurface::initPhysics()
{
	btCollisionShape* groundShape = new btBoxShape(btVector3(50, 3, 50));
	_simEngine->addCollisionShape(groundShape);
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

	_simEngine->addCollisionShape(groundShape);

	//create ground object
	_simEngine->localCreateRigidBody(0, tr, groundShape);
}

GroundSurface::GroundSurface(SimEngine *engine)
: m_indexVertexArrays(0), m_vertices(0), _simEngine(engine)
{
}

GroundSurface::~GroundSurface() {
	delete m_indexVertexArrays;
	delete m_vertices;
}

////////////////////////////////////
// VehicleDemo class
////////////////////////////////////

void VehicleDemo::keyboardCallback(unsigned char key, int x, int y) {
	(void) x;
	(void) y;

	switch (key) {
	case 'q':
		exit(0);
		break;
	case 'h':	_simEngine->setDebug(btIDebugDraw::DBG_NoHelpText);	    break;
	case 'w':	_simEngine->setDebug(btIDebugDraw::DBG_DrawWireframe);  break;
	case 'p':	_simEngine->setDebug(btIDebugDraw::DBG_ProfileTimings);	break;
	case '=': {
		int maxSerializeBufferSize = 1024 * 1024 * 5;
		btDefaultSerializer* serializer = new btDefaultSerializer(
				maxSerializeBufferSize);
		//serializer->setSerializationFlags(BT_SERIALIZE_NO_DUPLICATE_ASSERT);
		_simEngine->dynamicsWorld()->serialize(serializer);
		FILE* f2 = fopen("testFile.bullet", "wb");
		fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(),
				1, f2);
		fclose(f2);
		delete serializer;
		break;
	}
	case 'm': _simEngine->setDebug(btIDebugDraw::DBG_EnableSatComparison);  break;
	case 'n':	_simEngine->setDebug(btIDebugDraw::DBG_DisableBulletLCP);		  break;
	case 't':	_simEngine->setDebug(btIDebugDraw::DBG_DrawText);		          break;
	case 'y':	_simEngine->setDebug(btIDebugDraw::DBG_DrawFeaturesText);		  break;
	case 'a':	_simEngine->setDebug(btIDebugDraw::DBG_DrawAabb);             break;
	case 'c':	_simEngine->setDebug(btIDebugDraw::DBG_DrawContactPoints);    break;
	case 'C':	_simEngine->setDebug(btIDebugDraw::DBG_DrawConstraints);      break;
	case 'L':	_simEngine->setDebug(btIDebugDraw::DBG_DrawConstraintLimits); break;

	case 'd':
		_simEngine->setDebug(btIDebugDraw::DBG_NoDeactivation);
		if (_simEngine->debugMode() & btIDebugDraw::DBG_NoDeactivation) {
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
	case '1':	_simEngine->setDebug(btIDebugDraw::DBG_EnableCCD); break;

	default:
		//        std::cout << "unused key : " << key << std::endl;
		break;
	}

	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(_simEngine->debugMode());

}

void VehicleDemo::setDebugMode(int mode) {
	_simEngine->debugMode(mode);
	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(mode);
}

VehicleDemo::VehicleDemo() :
	_vehicle(0), _ground(0),
 _camera(0), m_cameraHeight(4.f),
	m_minCameraDistance(3.f), m_maxCameraDistance(10.f)
{
}

VehicleDemo::~VehicleDemo() {
	if (_simEngine)
		delete _simEngine;

	if (_ground)
		delete _ground;

	if (_vehicle)
		delete _vehicle;

	if (_camera)
		delete _camera;
}

void VehicleDemo::initPhysics() {

	// set up the simulation
	_simEngine = new SimEngine();
	_simEngine->initPhysics();

	// Set up the ground
	_ground = new GroundSurface(_simEngine);
	_ground->initPhysics();

	// Set up the vehicle
	_vehicle = new Vehicle(_simEngine);
	_vehicle->initPhysics();

	// Set up the camera
	_camera = new GlutCamera(_simEngine);
	_camera->setCameraPosition(btVector3(30, 30, 30));
	_camera->setCameraDistance(26.f);

	// Connect the debug drawer
	_simEngine->dynamicsWorld()->setDebugDrawer(_camera->debugDrawer());
}

//to be implemented by the demo
void VehicleDemo::renderme() {
	updateCamera();

	btVector3 worldBoundsMin, worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,
			worldBoundsMax);

	_vehicle->drawWheels(camera()->shapeDrawer(), worldBoundsMin, worldBoundsMax);

	_camera->renderme(_simEngine->debugMode());
}

void VehicleDemo::clientMoveAndDisplay() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// apply controls
	_vehicle->move();

	_simEngine->stepSimulation();

	renderme();

	//optional but useful: debug drawing
	_simEngine->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}

void VehicleDemo::displayCallback(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	_simEngine->debugDrawWorld();

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

