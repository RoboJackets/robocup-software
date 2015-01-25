#include "PassReceiver.hpp"
#include "Utils.hpp"
#include <math.h>
#include <iostream>



using namespace Geometry2d;
using namespace std;


namespace Gameplay { REGISTER_CONFIGURABLE(PassReceiver) }



// ConfigDouble *Gameplay::PassReceiver::_relativeMouthAngleValue;
ConfigDouble *Gameplay::PassReceiver::_backoffDistance;
ConfigDouble *Gameplay::PassReceiver::_maxExecutionError;
ConfigDouble *Gameplay::PassReceiver::_kickTimeout;

ConfigBool *Gameplay::PassReceiver::_debug;

void Gameplay::PassReceiver::createConfiguration(Configuration *cfg)
{
	// _relativeMouthAngleValue = new ConfigDouble(cfg, "PassReceiver/Moutn Angle", 0);

	_backoffDistance = new ConfigDouble(cfg, "PassReceiver/Backoff Distance", 0.078674155);
	_maxExecutionError = new ConfigDouble(cfg, "PassReceiver/Max Execution Error", 0.03);
	_kickTimeout = new ConfigDouble(cfg, "PassReceiver/Kick Timeout", 2.0);

	_debug = new ConfigBool(cfg, "PassReceiver/Debug", true);
}

Gameplay::PassReceiver::PassReceiver(GameplayModule *gameplay)
	: ActionBehavior(gameplay),
	  _state(Positioning),
	  _success(false),
	  _kickStartTime(0)
{
	// _relativeMouthAngle = 0;
}

bool Gameplay::PassReceiver::isSettingUp()
{
	return _state == Positioning;
}

bool Gameplay::PassReceiver::isSetup()
{
	return _state == Positioned;
}

bool Gameplay::PassReceiver::isActing()
{
	return _state == Receiving;// || _state == Positioning || _state == Positioned;
}

bool Gameplay::PassReceiver::isDone()
{
	return _state == Completed || _state == Failed;
}

void Gameplay::PassReceiver::restart()
{
	_state = Positioning;
	_success = false;
	_kickStartTime = 0;
}


Geometry2d::Point Gameplay::PassReceiver::closestPointOnRayToPoint(Point &rayStart, float rayAngle, Point &pt) {

	//	make a really long line segment that starts at rayStart and is angled at rayAngle
	const float projectionDist = 200;
	Point rayEnd;
	rayEnd.x = rayStart.x + projectionDist * cosf(rayAngle);
	rayEnd.y = rayStart.y + projectionDist * sinf(rayAngle);

	Line line(rayStart, rayEnd);
	return line.nearestPoint(pt);
}


void Gameplay::PassReceiver::getPassRay(Point &startPt, float &angle) {
	if ( passStarted() ) {
		startPt = ball().pos;
        
        Point delta = ball().vel;
		angle = delta.angle();

	} else {
		startPt = ball().pos;
        
        Point delta = (actionTarget - ball().pos);
		angle = delta.angle();
	}
}



// float Gameplay::PassReceiver::targetGlobalMouthAngle() {
// 	Point pt;
// 	float angle;
// 	getPassRay(pt, angle);
// 	return angle + *_relativeMouthAngleValue;
// }

//	returns position that the mouth should be at
Point Gameplay::PassReceiver::targetMouthPosition() {
	bool ballInFlight = _kickStartTime > 0;

	if ( ballInFlight ) {
		Point rayPt;
		float rayAngle;
		getPassRay(rayPt, rayAngle);

		Point target = closestPointOnRayToPoint(rayPt, rayAngle, robot->pos);

		// float backoffAngle = //////////////////////////////	//	FIXME: implement
		// Point backoffVector(*_backoffDistance * cosf(backoffAngle), *_backoffDistance * sinf(backoffAngle));


		return target;
	} else {
    	return actionTarget;
    }
}



Point Gameplay::PassReceiver::currentMouthPosition() {
	Point center = robot->pos;

	Point offset(Robot_Radius, 0);
	offset.rotate(robot->angle);

	return center + offset;
}


//	returns  the position that the center of the robot should be at
Point Gameplay::PassReceiver::targetCenterPosition() {
	Point mouth = targetMouthPosition();
	return mouth;
}


float Gameplay::PassReceiver::receivePositionError() {
	float dist = (targetCenterPosition() - robot->pos).mag();
	// float distAngle = Line(receivePosition(), ball().pos).distTo(robot->pos);
	//return std::max(dist, distAngle);

	return dist;
}

bool Gameplay::PassReceiver::isAtReceivePosition()
{
	return receivePositionError() < *_maxExecutionError;
}

bool Gameplay::PassReceiver::passStarted() {
	return _kickStartTime != 0;
}

bool Gameplay::PassReceiver::passTimedOut() {

	if ( _kickStartTime == 0 ) return false;
	return timestamp() - _kickStartTime > *_kickTimeout * 1000000;  // Micro-seconds
}

bool Gameplay::PassReceiver::ballBehindRobot() {

	return false;	//	FIXME


	Point passVector = robot->pos - kickPoint;
	Point botToBall = ball().pos - robot->pos;

	//	rotate the vector so that if the ball is past the robot, delta will have a positive x value
	Point delta = botToBall;
	delta.rotate(robot->pos, -(passVector.angle()));


	bool behind = delta.x > -Robot_Radius / 2.0f;
	return behind;
}


bool Gameplay::PassReceiver::passMissed() {
	bool missed = (_kickStartTime > 0 && ballBehindRobot()) || passTimedOut();

	if ( missed && *_debug ) cout << "Pass receiver missed :(" << endl;

	return missed;
}



void Gameplay::PassReceiver::partnerDidKick() {
	_kickStartTime = timestamp();
	_state = Receiving;

	if ( *_debug ) cout << "PassReceiver was told of a kick" << endl;
}



bool Gameplay::PassReceiver::run()
{
	using namespace Geometry2d;

	if (!robot || !robot->visible) {
		return false;
	}



	//	if we're done, we're done
	if ( _state == Completed || _state == Failed ) return false;


	if ( _state == Positioning ) {
		if ( isAtReceivePosition() ) _state = Positioned;
	} else if ( _state == Positioned ) {
		if ( !isAtReceivePosition() ) _state = Positioning;
	}

	//	call partnerDidKick() if necessary
    if( partner && partner->isDone() && !passStarted() )
        partnerDidKick();

	if ( passStarted() ) _state = Receiving;



	if ( _state == Receiving ) {
		if ( robot->hasBall() ) {
			_state = Completed;
			return false;
		} else if ( passMissed() ) {
			_state = Failed;
			return false;
		}
	}


	// Debug visualization
	if (*_debug) {
		if (_state == Positioning) {
			robot->addText(QString("DR: Positioning"), Qt::yellow, QString("PassReceiver"));
			robot->addText(QString("DR error: %1").arg(receivePositionError()), Qt::yellow, QString("PassReceiver"));
			// Draw circle at pass position
			state()->drawCircle(targetCenterPosition(), Robot_Radius + 0.05, Qt::yellow, QString("PassReceiver"));
		} else if (_state == Positioned) {
			robot->addText(QString("DR: Positioned"), Qt::yellow, QString("PassReceiver"));
			// Draw circle at pass position
			state()->drawCircle(targetCenterPosition(), Robot_Radius + 0.05, Qt::yellow, QString("PassReceiver"));
			robot->addText(QString("DR error: %1").arg(receivePositionError()), Qt::yellow, QString("PassReceiver"));
			// Draw line to show where we think the ball is going to travel
			state()->drawLine(ball().pos, targetMouthPosition(), Qt::yellow, QString("PassReceiver"));
		} else if (_state == Receiving) {
			robot->addText(QString("DR: Receiving"), Qt::green, QString("PassReceiver"));
			// robot->addText(QString("DR error: %1").arg(receivePositionError()), Qt::green, QString("PassReceiver"));
			// Draw circle at pass position
			state()->drawCircle(targetCenterPosition(), Robot_Radius + 0.05, Qt::green, QString("PassReceiver"));
			// Draw line to show where we think the ball is going to travel
			state()->drawLine(ball().pos, targetMouthPosition(), Qt::green, QString("PassReceiver"));
		} else if (_state == Completed) {
			// Do nothing
			robot->addText(QString("DR: Completed"), Qt::red, QString("PassReceiver"));
		} else if ( _state == Failed ) {
			robot->addText(QString("DR: Failed"), Qt::red, QString("PassReceiver"));
		}
	}



	if ( _state == Positioning ) {
		robot->resetAvoidBall();
	} else {
		robot->disableAvoidBall();
	}

	//	if we're positioning, receiving, or positioned...
	robot->move(targetCenterPosition(), false);
	robot->face(getFacePoint());


	return true;
}


Point Gameplay::PassReceiver::getFacePoint() {
	Point pt = ball().pos;
	//pt.rotate(robot->pos, *_relativeMouthAngleValue);
	return pt;
}

