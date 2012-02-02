#pragma once

#include <map>

#include <Physics/SimEngine.hpp>

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;
class GL_ShapeDrawer;

class btCollisionShape;
class btDynamicsWorld;

class SimpleVehicle;
class GroundSurface;
class GlutCamera;

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

