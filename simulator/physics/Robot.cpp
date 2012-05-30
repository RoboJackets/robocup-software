#include "Robot.hpp"
#include "Ball.hpp"
#include "Environment.hpp"
#include "GL_ShapeDrawer.h"
#include "RobotMotionState.hpp"
#include "RobotBallController.hpp"

#include <Utils.hpp>
#include <Constants.hpp>
#include <stdio.h>
#include <math.h>

#include <Geometry2d/TransformMatrix.hpp>

#include <boost/foreach.hpp>

using namespace Geometry2d;

//FIXME: parameters have no sensible interpretation
// static physics parameters
static const float maxEngineForce = 3000.f; //this should be engine/velocity dependent
static const float maxBreakingForce = 100.f;
static const float wheelFriction = 1000; //BT_LARGE_FLOAT;
static const float suspensionStiffness = 1000.f;
static const float suspensionDamping = 2.3f;
static const float suspensionCompression = 4.4f;
static const float rollInfluence = 0.0f;
static const btScalar suspensionRestLength = 0.05f*scaling;//0.6f
static const float robotWeight = 800;


Robot::Robot(Environment* env, unsigned int id,  Robot::RobotRevision rev, const Geometry2d::Point& startPos) :
			Entity(env), shell(id), _rev(rev), _lastKicked(0),_brakingForce(0),
			_robotChassis(0), _robotVehicle(0), _wheelShape(0), _simEngine(env->getSimEngine()),_controller(0)
{
	visibility = 100;
	ballSensorWorks = true;
	chargerWorks = true;

	_startTransform.setIdentity();
	_startTransform.setOrigin(btVector3(startPos.y,0,startPos.x)*scaling);
	setEngineForce(0);
}

Robot::~Robot()
{
	delete _vehicleRayCaster;
	delete _robotVehicle;
	delete _wheelShape;
	delete _controller;
}

void Robot::initPhysics(const bool& blue)
{
	// Assumes that Y is up
	int rightIndex = 0;
	int upIndex = 1;
	int forwardIndex = 2;
	btVector3 wheelDirectionCS0(0, -1, 0);
	btVector3 wheelAxleCS;

	// Create robot chassis
	btConvexHullShape* convexShape = new btConvexHullShape();
	int numPoints = 50;
	float PI = 3.14159265;
	float mrad = asin((Sim_Robot_MouthWidth/2.f)/Sim_Robot_Radius); //angle from mouth center to corner
	float rad_incr = 2*(PI-mrad)/(float)numPoints;
	float angle = mrad;
	for(int i=0; i<=numPoints; i++){
		//top and bottom points at angle around shell
		btVector3* pt = new btVector3(Sim_Robot_Radius*sin(angle),Sim_Robot_Height/2.f,Sim_Robot_Radius*cos(angle));
		btVector3* pb = new btVector3(Sim_Robot_Radius*sin(angle),-Sim_Robot_Height/2.f,Sim_Robot_Radius*cos(angle));
		convexShape->addPoint(*pt);
		convexShape->addPoint(*pb);
		angle += rad_incr;
	}
	convexShape->initializePolyhedralFeatures();
	convexShape->setMargin(0.004*scaling);
	_simEngine->addCollisionShape((btCollisionShape*)convexShape);

	// Chassis color
	btVector3* color = new btVector3(1.f,1.f,0.5f);
	if(blue)
		*color = btVector3(0.f,0.f,1.f);
	convexShape->setUserPointer(color);

	// Init spawn location of robot in engine WS
	if(blue)
		_startTransform.setRotation(btQuaternion(btVector3(0,1,0),PI));
	float connectionHeight = -0.01f*scaling;//Wheel connection height
	btVector3 robotCenter = btVector3(0, Sim_Robot_Height/2.f-connectionHeight, 0);
	_startTransform.setOrigin(robotCenter+_startTransform.getOrigin());

	// Create robot chassis rigid body
	_robotChassis = _simEngine->localCreateRobot(robotWeight, _startTransform, convexShape);

	// TODO: init robot ball controller
	_controller = new RobotBallController(this);
	_controller->initPhysics();

	((RobotMotionState*)_robotChassis->getMotionState())->m_controller = _controller;

	// Create wheel for rendering
	_wheelShape = new btCylinderShapeX(
			btVector3(Sim_Wheel_Width/2.f, Sim_Wheel_Radius, Sim_Wheel_Radius));

	// Create raycast vehicle
	{
		_vehicleRayCaster = new btDefaultVehicleRaycaster(_simEngine->dynamicsWorld());
		_robotVehicle = new btRaycastVehicle(_tuning, _robotChassis, _vehicleRayCaster);

		// Never deactivate the vehicle
		_robotChassis->setActivationState(DISABLE_DEACTIVATION);

		_simEngine->addVehicle(_robotVehicle);

		bool isFrontWheel = false;

		// Choose coordinate system
		_robotVehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

		// Position omni-wheels // Chassis orientation corrected for automatically in addWheel()
		btVector3 connectPoint = btVector3(0,connectionHeight,Sim_Robot_Radius-Sim_Wheel_Width/2.f);
		btVector3 forward = btVector3(0,0,1.0);
		//FL
		btVector3 connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),PI/4.f*1);
		wheelAxleCS = forward.rotate(btVector3(0,1,0), PI/4.f*1);
		_robotVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, Sim_Wheel_Radius, _tuning, isFrontWheel);
		//FR
		connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),PI/4.f*-1);
		wheelAxleCS = forward.rotate(btVector3(0,1,0), PI/4.f*-1);
		_robotVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, Sim_Wheel_Radius, _tuning, isFrontWheel);
		//BR
		connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),PI/4.f*5);
				wheelAxleCS = forward.rotate(btVector3(0,1,0), PI/4.f*5);
		isFrontWheel = false;
		_robotVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS ,
				suspensionRestLength, Sim_Wheel_Radius, _tuning, isFrontWheel);
		//BL
		connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),PI/4.f*3);
				wheelAxleCS = forward.rotate(btVector3(0,1,0), 3.141/4.f*3);
		_robotVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, Sim_Wheel_Radius, _tuning, isFrontWheel);

		for (int i = 0; i < _robotVehicle->getNumWheels(); i++) {
			btWheelInfo& wheel = _robotVehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}

	_robotChassis->getBroadphaseProxy()->m_collisionFilterGroup;

	resetScene(); // force initial physics state - everything stationary
}

void Robot::position(float x, float y)
{
	btTransform trans;
	_robotChassis->getMotionState()->getWorldTransform(trans);
	trans.setOrigin(btVector3(y*scaling,trans.getOrigin().y(),x*scaling));//map field coord -> engine WS; x->z, y->x;

	_robotChassis->setCenterOfMassTransform(trans);
	_controller->syncMotionState(trans);

	if (_robotVehicle) {
		_robotVehicle->resetSuspension();
		for (int i = 0; i < _robotVehicle->getNumWheels(); i++) {
			//synchronize the wheels with the (interpolated) chassis worldtransform
			_robotVehicle->updateWheelTransform(i, true);
		}
	}
}

void Robot::velocity(float x, float y, float z)
{
	//Apply engine forces
}

Geometry2d::Point Robot::getPosition() const
{
	btTransform tr;
	getWorldTransform(tr);
	btVector3 pos = tr.getOrigin()/scaling;
	return Geometry2d::Point(pos.z(),pos.x());
}

float Robot::getAngle() const
{
	btTransform tr;
	getWorldTransform(tr);
	return tr.getRotation().getAngle();
}

void Robot::getWorldTransform(btTransform& chassisWorldTrans) const
{
	_robotChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
}

void Robot::radioTx(const Packet::RadioTx::Robot *data)
{
	static const float dt = 0.016; // msec per frame

	// Simple control: directly copy velocities

	// Update the position
	/*const Point body_tvel(data->body_x(), data->body_y());
	const TransformMatrix body_disp(dt * body_tvel, dt * data->body_w());
	const TransformMatrix cur_pose(_pos, _omega);

	TransformMatrix updated_pose = cur_pose * body_disp;
	_pos = updated_pose.origin();
	_theta = updated_pose.rotation();

	_vel  = body_tvel.rotated(_theta);
	_omega = data->body_w();*/

	/** How we kick:
	 * Kick speed will be zeroed if we are not kicking
	 * Otherwise we determine which direction we are kicking and kick that way, using
	 * max speeds guessed with science
	 */

	// FIXME: rework this section to use information that we actually have
//	if (data->kick() && (Utils::timestamp() - _lastKicked) > RechargeTime && chargerWorks)
//	{
//		// FIXME: make these parameters some place else
//		float maxKickSpeed = 5.0f, // m/s direct kicking speed
//				maxChipSpeed = 3.0f; // m/s chip kicking at the upwards angle
////				chipAngle = 20.0f;   // angle (degrees) of upwards chip
//
//		// determine the kick speed
//		float kickSpeed;
//		bool chip = data->use_chipper();
//		if (chip)
//		{
//			kickSpeed = data->kick() / 255.0f * maxChipSpeed;
//		} else {
//			kickSpeed = data->kick() / 255.0f * maxKickSpeed;
//		}
//	}
}

Packet::RadioRx Robot::radioRx() const
{
	Packet::RadioRx packet;

	// FIXME: packet format changed - construct the return packet more carefully
//	packet.set_timestamp(Utils::timestamp());
//	packet.set_battery(1.0f);
//	packet.set_rssi(1.0f);
//	packet.set_charged(chargerWorks && (Utils::timestamp() - _lastKicked) > RechargeTime);
//
//	BOOST_FOREACH(const Ball* ball, _env->balls())
//	{
//		packet.set_ball_sense(ballSense(ball) || !ballSensorWorks);
//	}

	return packet;
}

bool Robot::ballSense(const Ball *ball) const
{
	return false;
}

void Robot::applyEngineForces() {
	int wheelIndex = 2;
	_robotVehicle->applyEngineForce(_engineForce[wheelIndex], wheelIndex);
	_robotVehicle->setBrake(_brakingForce, wheelIndex);
	wheelIndex = 3;
	_robotVehicle->applyEngineForce(_engineForce[wheelIndex], wheelIndex);
	_robotVehicle->setBrake(_brakingForce, wheelIndex);

	wheelIndex = 0;
	_robotVehicle->applyEngineForce(_engineForce[wheelIndex], wheelIndex);
	_robotVehicle->setBrake(_brakingForce, wheelIndex);
	wheelIndex = 1;
	_robotVehicle->applyEngineForce(_engineForce[wheelIndex], wheelIndex);
	_robotVehicle->setBrake(_brakingForce, wheelIndex);
}

void Robot::renderWheels(GL_ShapeDrawer* shapeDrawer, const btVector3& worldBoundsMin, const btVector3& worldBoundsMax) const {
	btVector3 wheelColor(1, 0, 0);
	btScalar m[16];
	for (int i = 0; i < _robotVehicle->getNumWheels(); i++) {
		//synchronize the wheels with the (interpolated) chassis worldtransform
		_robotVehicle->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		_robotVehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
		int debug_mode = _simEngine->dynamicsWorld()->getDebugDrawer()->getDebugMode();
		shapeDrawer->drawOpenGL(m, _wheelShape, wheelColor, debug_mode,
				worldBoundsMin, worldBoundsMax);
	}
}

void Robot::resetScene() {
	setEngineForce(0);
	_robotChassis->setCenterOfMassTransform(((btDefaultMotionState* ) _robotChassis->getMotionState())->m_startWorldTrans);
	_robotChassis->setLinearVelocity(btVector3(0, 0, 0));
	_robotChassis->setAngularVelocity(btVector3(0, 0, 0));
	_simEngine->dynamicsWorld()->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
			_robotChassis->getBroadphaseHandle(), _simEngine->dynamicsWorld()->getDispatcher());
	if (_robotVehicle) {
		_robotVehicle->resetSuspension();
		for (int i = 0; i < _robotVehicle->getNumWheels(); i++) {
			//synchronize the wheels with the (interpolated) chassis worldtransform
			_robotVehicle->updateWheelTransform(i, true);
		}
	}
}

void Robot::steerLeft() {
	setEngineForce(maxEngineForce);
	_brakingForce = 0.f;
}

void Robot::steerRight() {
	setEngineForce(-maxEngineForce);
	_brakingForce = 0.f;
}

void Robot::driveForward() {
	_engineForce[0] = -maxEngineForce;
	_engineForce[1] = maxEngineForce;
	_engineForce[2] = maxEngineForce;
	_engineForce[3] = -maxEngineForce;
	_brakingForce = 0.f;
}

void Robot::driveBackward() {
	_brakingForce = maxBreakingForce;
	setEngineForce(0.f);
}
