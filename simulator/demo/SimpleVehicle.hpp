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

class GL_ShapeDrawer;
class btCollisionShape;
class btRigidBody;

class SimEngine;

class SimpleVehicle {
protected:

	// physics components
	btRigidBody* _carChassis;
	btRaycastVehicle::btVehicleTuning _tuning;
	btVehicleRaycaster* _vehicleRayCaster;
	btRaycastVehicle* _vehicle;
	btCollisionShape* _wheelShape;

	// control inputs
	float _engineForce;
	float _breakingForce;
	float _vehicleSteering;

	// links to the engine
	SimEngine *_simEngine;

public:

	SimpleVehicle(SimEngine* engine)
	: _carChassis(0), _vehicle(0), _wheelShape(0),
	  _simEngine(engine)
	{
		_engineForce = 0.f;
		_breakingForce = 0.f;
		_vehicleSteering = 0.f;
	}

	~SimpleVehicle() {
		delete _vehicleRayCaster;
		delete _vehicle;
		delete _wheelShape;
	}

	void initPhysics();

	void drawWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax);

	void move();

	void resetScene();

	void getWorldTransform(btTransform& chassisWorldTrans) const;

	// access
	void engineForce    (float val) { _engineForce     = val; }
	void breakingForce  (float val) { _breakingForce   = val; }
	void vehicleSteering(float val) { _vehicleSteering = val; }

	float engineForce    () const { return _engineForce;     }
	float breakingForce  () const { return _breakingForce;   }
	float vehicleSteering() const { return _vehicleSteering; }

	// bang-bang steering and throttle
	void steerLeft();
	void steerRight();
	void driveForward();
	void driveBackward();

};
