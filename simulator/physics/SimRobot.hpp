#pragma once

#include <physics/SimEngine.hpp>
#include <physics/PhysicsConstants.hpp>
#include "GL_ShapeDrawer.h"

class SimRobot {

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

	SimRobot(SimEngine* engine)
	: _carChassis(0), _vehicle(0), _wheelShape(0),
	  _simEngine(engine)
	{
		_engineForce = new float[4];
		engineForce(0.f);//
		_breakingForce = 0.f;
		_vehicleSteering = 0.f;
	}

	~SimRobot() {
		delete _vehicleRayCaster;
		delete _vehicle;
		delete _wheelShape;
		delete _engineForce;
	}

	void initPhysics(bool blue, const btVector3& pos);

	void drawWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax);

	void move();

	void resetScene();

	void getWorldTransform(btTransform& chassisWorldTrans) const;

	//utility
	void position(float x, float y);

	// access
	void engineForce    (int wheelIndex, float val) { _engineForce[wheelIndex] = val; }
	void engineForce 	(float val) { for(int i=0; i<4; i++){ _engineForce[i] = val; } }
	void breakingForce  (float val) { _breakingForce   = val; }
	void vehicleSteering(float val) { _vehicleSteering = val; }

	float* engineForce   () const { return _engineForce;     }
	float breakingForce  () const { return _breakingForce;   }
	float vehicleSteering() const { return _vehicleSteering; }
	btRigidBody* carChassis() const { return _carChassis; }
	btRaycastVehicle* vehicle() const { return _vehicle; }

	// bang-bang steering and throttle
	void steerLeft();
	void steerRight();
	void driveForward();
	void driveBackward();
};
