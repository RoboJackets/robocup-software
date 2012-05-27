#pragma once

#include "Robot.hpp"

#include <physics/SimEngine.hpp>
#include <physics/PhysicsConstants.hpp>
#include "GL_ShapeDrawer.h"

class SimRobot : public Robot
{
protected:
	// physics components
	btRigidBody* _carChassis;
	btRaycastVehicle::btVehicleTuning _tuning;
	btVehicleRaycaster* _vehicleRayCaster;
	btRaycastVehicle* _vehicle;
	btCollisionShape* _wheelShape;

	// control inputs
	float _engineForce[4];
	float _breakingForce;

	// links to the engine
	SimEngine *_simEngine;

public:

	SimRobot(Environment* env, unsigned int id, Robot::RobotRevision rev, const Geometry2d::Point& pos, SimEngine* engine) :
		Robot(env,id,rev,pos), _carChassis(0), _vehicle(0), _wheelShape(0), _simEngine(engine)
	{
		engineForce(0);
		_breakingForce = 0.f;
	}

	~SimRobot() {
		delete _vehicleRayCaster;
		delete _vehicle;
		delete _wheelShape;
	}

public:
	//Entity interface
	virtual void position(float x, float y); // world coords
	virtual void velocity(float x, float y, float w); // body coords
	virtual Geometry2d::Point getPosition() const;
	virtual float getAngle() const;

public:

	void initPhysics(const bool& blue);

	void drawWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax);

	void move();

	void resetScene();

	void getWorldTransform(btTransform& chassisWorldTrans) const;

	// access
	void engineForce    (int wheelIndex, float val) { _engineForce[wheelIndex] = val; }
	void engineForce 	(float val) { for(int i=0; i<4; i++){ _engineForce[i] = val; } }
	void breakingForce  (float val) { _breakingForce   = val; }

	const float* engineForce   () const { return _engineForce; }
	float breakingForce  () const { return _breakingForce; }
	btRigidBody* carChassis() const { return _carChassis; }

	// bang-bang steering and throttle
	void steerLeft();
	void steerRight();
	void driveForward();
	void driveBackward();
};
