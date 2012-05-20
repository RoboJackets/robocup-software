#pragma once

#include <physics/SimEngine.hpp>

class SimpleVehicle {

	//Wheel indices
	static const int FL = 0;
	static const int FR = 1;
	static const int BR = 2;
	static const int BL = 3;

protected:

	// physics components
	btRigidBody* _carChassis;
	btRaycastVehicle::btVehicleTuning _tuning;
	btVehicleRaycaster* _vehicleRayCaster;
	btRaycastVehicle* _vehicle;
	btCollisionShape* _wheelShape;

	// control inputs
	float* _engineForce;
	float _breakingForce;
	float _vehicleSteering;

	// links to the engine
	SimEngine *_simEngine;

public:

	SimpleVehicle(SimEngine* engine)
	: _carChassis(0), _vehicle(0), _wheelShape(0),
	  _simEngine(engine)
	{
		_engineForce = new float[4];
		engineForce(0.f);//
		_breakingForce = 0.f;
		_vehicleSteering = 0.f;
	}

	~SimpleVehicle() {
		delete _vehicleRayCaster;
		delete _vehicle;
		delete _wheelShape;
		delete _engineForce;
	}

	void initPhysics();

	void drawWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax);

	void move();

	void resetScene();

	void getWorldTransform(btTransform& chassisWorldTrans) const;

	// access
	void engineForce    (int wheelIndex, float val) { _engineForce[wheelIndex] = val; }
	void engineForce 	(float val) { for(int i=0; i<4; i++){ _engineForce[i] = val; } }
	void breakingForce  (float val) { _breakingForce   = val; }
	void vehicleSteering(float val) { _vehicleSteering = val; }

	float* engineForce   () const { return _engineForce;     }
	float breakingForce  () const { return _breakingForce;   }
	float vehicleSteering() const { return _vehicleSteering; }
	btRigidBody* carChassis() const { return _carChassis; }

	// bang-bang steering and throttle
	void steerLeft();
	void steerRight();
	void driveForward();
	void driveBackward();
};
