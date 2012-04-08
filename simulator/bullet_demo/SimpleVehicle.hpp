#pragma once

#include <physics/SimEngine.hpp>

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
