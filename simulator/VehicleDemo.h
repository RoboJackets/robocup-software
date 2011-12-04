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

#include <map>

#include <BulletDynamics/Vehicle/btRaycastVehicle.h>

#include <GlutDemoApplication.h>

class VehicleDemo;

class Vehicle {
public:

	// physics components
	btRigidBody* m_carChassis;
	btRaycastVehicle::btVehicleTuning m_tuning;
	btVehicleRaycaster* m_vehicleRayCaster;
	btRaycastVehicle* m_vehicle;
	btCollisionShape* m_wheelShape;

	// links to the engine
	btDynamicsWorld*		m_dynamicsWorld;
	btAlignedObjectArray<btCollisionShape*>* m_collisionShapes;

	Vehicle(btDynamicsWorld* world, btAlignedObjectArray<btCollisionShape*>* collision_shapes)
	: m_carChassis(0), m_vehicle(0), m_wheelShape(0),
	  m_dynamicsWorld(world), m_collisionShapes(collision_shapes)
	{}

	~Vehicle() {
		delete m_vehicleRayCaster;
		delete m_vehicle;
		delete m_wheelShape;
	}

	void initPhysics(VehicleDemo* env);

	void drawWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax);

	void move();

	void resetScene();

	void getWorldTransform(btTransform& chassisWorldTrans) const;

};

///VehicleDemo shows how to setup and use the built-in raycast vehicle
class VehicleDemo: public GlutDemoApplication {
public:

//	// VehicleParts
//	btRigidBody* m_carChassis;
//	btRaycastVehicle::btVehicleTuning m_tuning;
//	btVehicleRaycaster* m_vehicleRayCaster;
//	btRaycastVehicle* m_vehicle;
//	btCollisionShape* m_wheelShape;
	Vehicle* _vehicle;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	// Environment parts
	class btBroadphaseInterface* m_overlappingPairCache;
	class btCollisionDispatcher* m_dispatcher;
	class btConstraintSolver* m_constraintSolver;
	class btDefaultCollisionConfiguration* m_collisionConfiguration;
	class btTriangleIndexVertexArray* m_indexVertexArrays;

	// ground triangle vertices
	btVector3* m_vertices;

	// Camera components
	float m_cameraHeight;
	float m_minCameraDistance;
	float m_maxCameraDistance;

	VehicleDemo();

	virtual ~VehicleDemo();

	virtual void clientMoveAndDisplay();

	virtual void clientResetScene();

	virtual void displayCallback();

	///a very basic camera following the vehicle
	virtual void updateCamera();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	void renderme();

	void initPhysics();

	// physics initializations for objects
	std::pair<btCollisionShape*, btTransform> addGround();
	void addVehicle(btDynamicsWorld* m_dynamicsWorld,
			btAlignedObjectArray<btCollisionShape*>& m_collisionShapes);

	// rendering
//	void drawWheels(const btVector3& worldBoundsMin, const btVector3& worldBoundsMax);

	static DemoApplication* Create() {
		VehicleDemo* demo = new VehicleDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
};

#endif //VEHICLE_DEMO_H
