#include "Robot.hpp"
#include "Ball.hpp"
#include "Environment.hpp"
#include "GL_ShapeDrawer.h"
#include "RobotMotionState.hpp"
#include "RobotBallController.hpp"

#include <Utils.hpp>
#include <stdio.h>
#include <math.h>

#include <Geometry2d/TransformMatrix.hpp>

#include <boost/foreach.hpp>

using namespace Geometry2d;

//FIXME: parameters have no sensible interpretation
// static physics parameters
static const float maxEngineForce = 2000.f; //this should be engine/velocity dependent
static const float maxBreakingForce = 100.f;
static const float wheelFriction = 1000; //BT_LARGE_FLOAT;
static const float suspensionStiffness = 1000.f;
static const float suspensionDamping = 2.3f;
static const float suspensionCompression = 4.4f;
static const float rollInfluence = 0.0f;
static const btScalar suspensionRestLength = 0.05f*scaling;//0.6f
static const float robotWeight = 8*scaling;


Robot::Robot(Environment* env, unsigned int id,  Robot::RobotRevision rev, const Geometry2d::Point& startPos) :
	Entity(env),shell(id), _rev(rev),
	_robotChassis(0), _robotVehicle(0), _wheelShape(0),_controller(0),
	_brakingForce(0),_targetVel(0,0,0),_targetRot(0),
	_simEngine(env->getSimEngine())
{
	visibility = 100;

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
	float mrad = asin((Sim_Robot_MouthWidth/2.f)/Sim_Robot_Radius); //angle from mouth center to corner
	float rad_incr = 2*(M_PI-mrad)/(float)numPoints;
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
		_startTransform.setRotation(btQuaternion(btVector3(0,1,0),M_PI));
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
		_robotVehicle = new RaycastVehicle(_tuning, _robotChassis, _vehicleRayCaster);

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
		btVector3 connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),M_PI/4.f*1);
		wheelAxleCS = forward.rotate(btVector3(0,1,0), M_PI/4.f*1);
		_robotVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, Sim_Wheel_Radius, _tuning, isFrontWheel);
		//FR
		connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),M_PI/4.f*-1);
		wheelAxleCS = forward.rotate(btVector3(0,1,0), M_PI/4.f*-1);
		_robotVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, Sim_Wheel_Radius, _tuning, isFrontWheel);
		//BR
		connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),M_PI/4.f*5);
				wheelAxleCS = forward.rotate(btVector3(0,1,0), M_PI/4.f*5);
		isFrontWheel = false;
		_robotVehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS ,
				suspensionRestLength, Sim_Wheel_Radius, _tuning, isFrontWheel);
		//BL
		connectionPointCS0 = connectPoint.rotate(btVector3(0,1,0),M_PI/4.f*3);
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


	//Allow rotation only on y-axis
	_robotChassis->setAngularFactor(btVector3(0,1,0));

	//Disable collisions with the ball controller
	_robotChassis->getBroadphaseHandle()->m_collisionFilterMask ^= btBroadphaseProxy::SensorTrigger;

	_robotChassis->setRestitution(0); //Disable bouncing

	_robotChassis->setDamping(0.95f,0.95f); //mimic friction

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

void Robot::velocity(float x, float y, float w)
{
	_targetVel = btVector3(y,0,x)*scaling;
	_targetRot = w;
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
	return tr.getRotation().getAxis()[1]*tr.getRotation().getAngle();
}

Geometry2d::Point Robot::getVelFS() const
{
	return Geometry2d::Point(_robotChassis->getLinearVelocity().z()/scaling,_robotChassis->getLinearVelocity().x()/scaling);
}

float Robot::getAngVelFS() const
{
	return _robotChassis->getAngularVelocity().y();
}

Geometry2d::Point Robot::getTargetVelFS() const
{
	return Geometry2d::Point(_targetVel.z()/scaling,_targetVel.x()/scaling);
}

float Robot::getTargetAngVelFS() const
{
	return _targetRot;
}

void Robot::getWorldTransform(btTransform& chassisWorldTrans) const
{
	_robotChassis->getMotionState()->getWorldTransform(chassisWorldTrans);
}

void Robot::radioTx(const Packet::RadioTx::Robot *data)
{
	velocity(data->body_x(),data->body_y(), data->body_w());
	if(data->kick()){
		_controller->prepareKick(data->kick(),data->use_chipper());
	}
	_controller->prepareDribbler(data->dribbler());
}

Packet::RadioRx Robot::radioRx() const
{
	Packet::RadioRx packet;

	packet.set_timestamp(timestamp());
	packet.set_battery(15.0f);
	packet.set_rssi(1.0f);
	packet.set_kicker_status(_controller->getKickerStatus());

	// FIXME: No.
	packet.set_ball_sense_status((_controller->hasBall() || !_controller->ballSensorWorks) ? Packet::HasBall : Packet::NoBall);

	// assume all motors working
	for (size_t i=0; i<5; ++i)
	{
		packet.add_motor_status(Packet::Good);
	}

	if (_rev == rev2008)
	{
		packet.set_hardware_version(Packet::RJ2008);
	} else if (_rev == rev2011) // FIXME: change to actual 2011
	{
		packet.set_hardware_version(Packet::RJ2011);
	} else
	{
		packet.set_hardware_version(Packet::Unknown);
	}

	return packet;
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

void Robot::applyEngineForces(float deltaTime) {

	// use this for finite acceleration, i.e. non-instantaneous
#define USE_ENGINE 1
#ifdef USE_ENGINE
	if(_targetVel.length() < SIMD_EPSILON && _targetRot == 0){

		for(int i=0; i<4; i++){
			_engineForce[i] = 0;
		}

	}else{
		//

//		btTransform worldTr = _robotChassis->getWorldTransform();
		btQuaternion worldRot = _robotChassis->getOrientation();

//		printf("WS pos = (%5.3f,%5.3f,%5.3f)\n", worldTr.getOrigin()[0],worldTr.getOrigin()[1],worldTr.getOrigin()[2]);
//		printf("WS rot = %5.3f\n", worldRot.getAngle());

		//basis vectors for robot - directions of driving two opposite wheels "forward", i.e. towards z in CS
		//f right as in northeast, f left as in northwest
		btVector3 forwardRightCS =  btVector3(-1,0,1).normalize(); // positive x is left b/c y is up, very tricky
		btVector3 forwardLeftCS  =  btVector3(1,0,1).normalize();

//		printf("forwardRightCS = (%5.3f,%5.3f,%5.3f)\n",forwardRightCS[0],forwardRightCS[1],forwardRightCS[2]);
//		printf("forwardLeftCS = (%5.3f,%5.3f,%5.3f)\n",forwardLeftCS[0],forwardLeftCS[1],forwardLeftCS[2]);


		btVector3 robotVel = _robotChassis->getLinearVelocity();
		robotVel.setY(0);

//		printf("WS velocity  = (%5.3f,%5.3f,%5.3f)\n", robotVel[0], robotVel[1], robotVel[2]);

		robotVel = robotVel.rotate(worldRot.getAxis(),-worldRot.getAngle());//undo world rotation

//		printf("CS velocity = (%5.3f,%5.3f,%5.3f)\n", robotVel[0], robotVel[1], robotVel[2]);

		btScalar  robotRot(_robotChassis->getAngularVelocity().getY());

//		printf("WS ang vel = %5.3f\n",robotRot);

		//velocity delta projected onto forwardRight and forwardLeft
		float velFR = (_targetVel-robotVel).dot(forwardRightCS);
		float velFL = (_targetVel-robotVel).dot(forwardLeftCS);

//		printf("error: %f\n",(_targetVel-robotVel).length());

		float angFR = velFR/Sim_Wheel_Radius;
		float angFL = velFL/Sim_Wheel_Radius;

		float forceFR = angFR;///deltaTime;
		float forceFL = angFL;///deltaTime;

		//need to drive at max engine force to achieve target velocity

		float rotVel  = (_targetRot-robotRot)*(Sim_Robot_Radius-Sim_Wheel_Width/2.f); //move to actual wheel loc
		//printf("target rot = %5.3f\n",_targetRot);

		float angVel  = rotVel/Sim_Wheel_Radius;

		float forceRot = angVel;///deltaTime;

		//assign translational forces
		//extremely quirky: all wheels turn counterclockwise w/ axis towards robot center
		_engineForce[FrontLeft]  = -forceFR/2;
		_engineForce[FrontRight] = forceFL/2;

		_engineForce[BackRight]  = forceFR/2;
		_engineForce[BackLeft]   = -forceFL/2;

		//multiply by mass to cancel /mass out later in raycastvehicle
		for(int i=0; i<4; i++){
			_engineForce[i] *= robotWeight;
		}

		//assign rotational forces
		for(int i=0; i<4; i++){
			_engineForce[i] += forceRot/(4.f*_robotChassis->getInvInertiaDiagLocal().y());//cancel out inertia factors later
		}

		//scale forces
		float max = 0;
		float force;
		for(int i=0; i<4; i++){
			force = _engineForce[i] > 0 ? _engineForce[i] : -_engineForce[i];
			if(force>maxEngineForce)
				if(force>max)
					max = force;
		}

		if(max){
//			printf("Maximum exceeded!\n");
			for(int i=0; i<4; i++)
				_engineForce[i] *= maxEngineForce/max;
		}
	}

	//apply forces
	_robotVehicle->applyEngineForce(_engineForce[FrontLeft], FrontLeft);
	_robotVehicle->setBrake(_brakingForce, FrontLeft);

	_robotVehicle->applyEngineForce(_engineForce[FrontRight], FrontRight);
	_robotVehicle->setBrake(_brakingForce, FrontRight);

	_robotVehicle->applyEngineForce(_engineForce[BackRight], BackRight);
	_robotVehicle->setBrake(_brakingForce, BackRight);

	_robotVehicle->applyEngineForce(_engineForce[BackLeft], BackLeft);
	_robotVehicle->setBrake(_brakingForce, BackLeft);
#else
	btVector3 vel(_targetVel);
	btTransform trans;
	_robotChassis->getMotionState()->getWorldTransform(trans);
	vel = vel.rotate(trans.getRotation().getAxis(),
						trans.getRotation().getAngle());
	_robotChassis->setLinearVelocity(vel);
	_robotChassis->setAngularVelocity(btVector3(0,1,0)*_targetRot);
#endif
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
