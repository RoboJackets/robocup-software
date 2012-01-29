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
#ifndef VEHICLE_DEMO_H
#define VEHICLE_DEMO_H

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;
class GL_ShapeDrawer;

#include "GlutCamera.hpp"
#include "Physics/SimEngine.hpp"

#include <map>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include "LinearMath/btQuickprof.h"

class btCollisionShape;
class btDynamicsWorld;
class btRigidBody;
class btTypedConstraint;

class VehicleDemo;

class Vehicle {
public:

	// physics components
	btRigidBody* m_carChassis;
	btRaycastVehicle::btVehicleTuning m_tuning;
	btVehicleRaycaster* m_vehicleRayCaster;
	btRaycastVehicle* m_vehicle;
	btCollisionShape* m_wheelShape;

	// control inputs
	float gEngineForce;
	float gBreakingForce;
	float gVehicleSteering;

	// links to the engine
	SimEngine *_simEngine;

	Vehicle(SimEngine* engine)
	: m_carChassis(0), m_vehicle(0), m_wheelShape(0),
	  _simEngine(engine)
	{
		gEngineForce = 0.f;
		gBreakingForce = 0.f;
		gVehicleSteering = 0.f;
	}

	~Vehicle() {
		delete m_vehicleRayCaster;
		delete m_vehicle;
		delete m_wheelShape;
	}

	void initPhysics();

	void drawWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax);

	void move();

	void resetScene();

	void getWorldTransform(btTransform& chassisWorldTrans) const;

	// bang-bang steering and throttle
	void steerLeft();
	void steerRight();
	void driveForward();
	void driveBackward();

};

class GroundSurface {
public:
	// Terrain components
	class btTriangleIndexVertexArray* m_indexVertexArrays;
	btVector3* m_vertices;

	// links to the engine
	SimEngine *_simEngine;

	GroundSurface(SimEngine *engine);

	~GroundSurface();

	void initPhysics();
};

///VehicleDemo shows how to setup and use the built-in raycast vehicle
class VehicleDemo {
public:

	// Drivable vehicle
	Vehicle* _vehicle;

	// Ground
	GroundSurface* _ground;

	// Dynamics/Collision Environment parts
	SimEngine* _simEngine;

	// Camera components
	GlutCamera* _camera;

	// Additional camera components
	float m_cameraHeight;
	float m_minCameraDistance;
	float m_maxCameraDistance;

	VehicleDemo();

	~VehicleDemo();

	btDynamicsWorld* getDynamicsWorld() {
		return _simEngine->dynamicsWorld();
	}

	void setDrawClusters(bool drawClusters) {}

	int getDebugMode() const {
		return _simEngine->debugMode();
	}

	GlutCamera* camera() { return _camera; }

	void setDebugMode(int mode);

	void clientMoveAndDisplay();

	void clientResetScene();

	void displayCallback();

	///a very basic camera following the vehicle
	void updateCamera();

	void specialKeyboard(int key, int x, int y);

	void specialKeyboardUp(int key, int x, int y);

	void renderme();

	void initPhysics();

	///glut callbacks

	void keyboardCallback(unsigned char key, int x, int y);

	void keyboardUpCallback(unsigned char key, int x, int y) {}

	// physics initializations for objects
	std::pair<btCollisionShape*, btTransform> addGround();

	void addVehicle(btDynamicsWorld* m_dynamicsWorld,
			btAlignedObjectArray<btCollisionShape*>& m_collisionShapes);
};

#endif //VEHICLE_DEMO_H
