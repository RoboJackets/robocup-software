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
#pragma once

#include <map>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include "LinearMath/btQuickprof.h"

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;
class GL_ShapeDrawer;

class btCollisionShape;
class btDynamicsWorld;

class SimpleVehicle;
class GroundSurface;
class GlutCamera;
class SimEngine;

///VehicleDemo shows how to setup and use the built-in raycast vehicle
class VehicleDemo {
protected:

	// Drivable vehicle
	SimpleVehicle* _vehicle;

	// Ground
	GroundSurface* _ground;

	// Dynamics/Collision Environment parts
	SimEngine* _simEngine;

	// Camera components
	GlutCamera* _camera;

	// Additional camera components
	float _cameraHeight;
	float _minCameraDistance;
	float _maxCameraDistance;

public:

	VehicleDemo();

	~VehicleDemo();

	btDynamicsWorld* getDynamicsWorld();

	void setDrawClusters(bool drawClusters) {}

	int getDebugMode() const;

	GlutCamera* camera() { return _camera; }

	void setDebugMode(int mode);

	void clientMoveAndDisplay();

	void clientResetScene();

	void displayCallback();

	/// a very basic camera following the vehicle
	void updateCamera();

	void specialKeyboard(int key, int x, int y);

	void specialKeyboardUp(int key, int x, int y);

	void renderme();

	void initPhysics();

	/// glut callbacks

	void keyboardCallback(unsigned char key, int x, int y);

	void keyboardUpCallback(unsigned char key, int x, int y) {}

	// physics initializations for objects
	std::pair<btCollisionShape*, btTransform> addGround();

	void addVehicle(btDynamicsWorld* m_dynamicsWorld,
			btAlignedObjectArray<btCollisionShape*>& m_collisionShapes);
};

