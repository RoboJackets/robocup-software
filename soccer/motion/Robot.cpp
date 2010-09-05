// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <motion/Robot.hpp>
#include <QMutexLocker>
#include <Geometry2d/Point.hpp>
#include <Utils.hpp>
#include <Constants.hpp>
#include <framework/RobotConfig.hpp>
#include <motion/Pid.hpp>

#include <iostream>
#include <stdio.h>

#include <algorithm>
#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;
using namespace Planning;

/** prints out a labeled point */
void printPt(const Geometry2d::Point& pt, const string& s="") {
	cout << s << ": (" << pt.x << ", " << pt.y << ")" << endl;
}

/** Constant for timestamp to seconds */
const float intTimeStampToFloat = 1000000.0f;

Motion::Robot::Robot(Configuration *config, SystemState *state, int shell) :
	_state(state),
	_self(state->self[shell]),
	_dynamics(state->self[shell]),
	_lastTimestamp(Utils::timestamp()),
	_w(0),
	_velFilter(Point(0.0, 0.0), 1),
	_wFilter(0.0, 1)
{
}

Motion::Robot::~Robot()
{
}

/**
 * Phases in motor commands:
 *  I:   Create a plan (RRT)
 *  II:  Create velocities
 *  III: Sanity Check velocities (obstacle avoidance, bounding)
 *  IV:  Smooth Velocities (Filter system)
 *  V:   Convert to motor commands
 */
void Motion::Robot::proc()
{
	bool verbose = false;
	_procMutex.lock();
	
	if (_self && _self->visible)
	{
		// set the correct PID parameters for angle
		_anglePid.kp = _self->config->motion.angle.p;
		_anglePid.ki = _self->config->motion.angle.i;
		_anglePid.kd = _self->config->motion.angle.d;

		// set coefficients for the output filters
		_velFilter.setCoeffs(_self->config->motion.output_coeffs.values());
		_wFilter.setCoeffs(_self->config->motion.output_coeffs.values());

		if (_state->gameState.state == GameState::Halt)
		{
			for (int i = 0; i < 4; ++i)
			{
				_self->radioTx.set_motors(i, 0);
			}
		}
		else
		{
			// record the type of planner for future use
			_plannerType = _self->cmd.planner;		// get the dynamics from the config

			// set the correct PID parameters for angle
			_anglePid.kp = _self->config->motion.angle.p;
			_anglePid.ki = _self->config->motion.angle.i;
			_anglePid.kd = _self->config->motion.angle.d;

			// set coefficients for the output filters
			_velFilter.setCoeffs(_self->config->motion.output_coeffs.values());
			_wFilter.setCoeffs(_self->config->motion.output_coeffs.values());

			// switch between planner types
			switch(_self->cmd.planner) {

			// handle direct velocity commands
			case MotionCmd::DirectVelocity:
			{
				// set the velocities from the gameplay module
				_vel = _self->cmd.direct_trans_vel;
				_w = _self->cmd.direct_ang_vel;

				break;
			}
			// handle direct motor commands - just pass on to wheel controller
			case MotionCmd::DirectMotor:
				break;

			// Catch force stop commands
			case MotionCmd::ForceStop:
			{
				const float deltaT = (_state->timestamp - _lastTimestamp)/intTimeStampToFloat;
				stop(deltaT);
			}

			// normal drive to point commands
			case MotionCmd::Point:
			{
				if (_self->cmd.pivot == MotionCmd::NoPivot)
				{
					genVelocity(_self->cmd.pathEnd);
				}
				else // handle pivot
				{
					//for now look at the ball
					_self->cmd.face = MotionCmd::Continuous;
					_self->cmd.goalOrientation = _self->cmd.pivotPoint;

					// create the velocities
					genVelocity(_self->cmd.pathEnd);
				}
				break;
			}
			}

			// scale velocities due to rules
			scaleVelocity();

			// sanity check the velocities due to obstacles
			const unsigned int LookAheadFrames = 5;
			sanityCheck(LookAheadFrames);

			// filter the velocities
			if (verbose) cout << "before - w: " << _w << " vel: (" << _vel.x << ", " << _vel.y << ")" << endl;
			_vel = _velFilter.filter(_vel);
			_w = _wFilter.filter(_w);
			if (verbose) cout << "after - w: " << _w << " vel: (" << _vel.x << ", " << _vel.y << ")" << endl;

			_self->cmd_vel = _vel;
			_self->cmd_w = _w;
		}
	}
	_lastTimestamp = _state->timestamp;

	_procMutex.unlock();
}

#if 0
//FIXME - Logging got rewritten and you can't get away with this anymore.
//  Either use debug drawing or get rid of it (need drawBezier in SystemState?)
void Robot::drawPath(QPainter& p)
{
	QMutexLocker ml(&_procMutex);

	// Draw a line from the robot directly to its goal
	p.setPen(Qt::darkRed);
	p.drawLine(_self->pos.toQPointF(), _self->cmd.goalPosition.toQPointF());

	_planner.draw(p);

	// Draw path
	p.setPen(Qt::red);
	const std::vector<Geometry2d::Point>& path = _path.points;
	const unsigned int n = path.size();
	for (unsigned int i = 1; i < n; ++i)
	{
		p.drawLine(path[i - 1].toQPointF(), path[i].toQPointF());
	}
}

void Robot::drawRRT(QPainter& p)
{
	// FIXME: this should be implemented at some point
}

void Robot::drawBezierTraj(QPainter& p) {

	QMutexLocker ml(&_procMutex);
	if (_plannerType == MotionCmd::Bezier && _bezierControls.size() > 0) {

		// create the coefficients
		vector<float> coeffs;
		size_t degree = _bezierControls.size();
		for (size_t i=0; i<degree; ++i) {
			coeffs.push_back(binomialCoefficient(degree-1, i));
		}

		// draw a curved blue line for the trajectory
		size_t nrBezierPts = 20;
		float inc = 1.0/nrBezierPts;
		Point prev = _bezierControls.front();
		Point pt;
		p.setPen(Qt::blue);
		for (size_t i = 0; i<nrBezierPts; ++i) {
			pt = evaluateBezier(inc * i, _bezierControls, coeffs);
			p.drawLine(prev.toQPointF(), pt.toQPointF());
			prev = pt;
		}
		p.drawLine(pt.toQPointF(), _bezierControls.back().toQPointF());
	}
}

void Robot::drawBezierControl(QPainter& p) {
	QMutexLocker ml(&_procMutex);
	if (_plannerType == MotionCmd::Bezier && _bezierControls.size() > 0) {

		// draw straight blue lines for control lines, and small circles for points
		Point prev(-1.0, -1.0);
		p.setPen(Qt::blue);
		BOOST_FOREACH(Point pt, _bezierControls) {
			p.drawEllipse(pt.toQPointF(), 10., 10.);
			if (prev.y != -1.0) {
				p.drawLine(prev.toQPointF(), pt.toQPointF());
			}
			prev = pt;
		}
	}
}
#endif

void Motion::Robot::stop(float dtime) {
	// handle rotation with PID - force value to zero
	_w = _anglePid.run(_w);

	// force to zero if close
	float ang_thresh = 0.2;
	if (_w < ang_thresh)
		_w = 0.0;
	
	//FIXME - Exploding numbers
	_w = 0.0;

	// get a model to use for approximating robot movement
	const float robotAngle = _self->angle;
	Dynamics::DynamicsInfo info = _dynamics.info(_vel.angle()*RadiansToDegrees - robotAngle, _w);

	// find magnitude of the maximum deceleration
	const float vv = info.deceleration * dtime;

	// find the magnitude of the current velocity
	const float vcur = _vel.mag();

	// if we can go straight to zero, do it, otherwise drop by maximum deceleration
	float vel_thresh = 0.5;
	if (vcur < vel_thresh)
		_vel = Geometry2d::Point(0.,0.);
	else
		_vel -= _vel.normalized()*vv;
}

void Motion::Robot::scaleVelocity() {
	// default scaling
	float vscale = 1;

	// pull out the robot angle
	const float robotAngle = _self->angle;

	// move slower in stopped mode
	if (_state->gameState.state == GameState::Stop)
	{
		vscale = .5;
	}

	// handle saturation of the velocity scaling
	_self->cmd.vScale = Utils::clamp(_self->cmd.vScale, 1.0, 0.0);

	// scale the velocity if necessary
	vscale *= _self->cmd.vScale;

	//max velocity info is in the new desired velocity direction
	Dynamics::DynamicsInfo info = _dynamics.info(_vel.angle() *
			RadiansToDegrees - robotAngle, _w);

	const float maxVel = info.velocity * vscale;

	// saturate translational velocity
	if (_vel.mag() > maxVel)
	{
		_vel = _vel.normalized() * maxVel;
	}

	// scale the rotational velocity
	_w *= _self->cmd.wScale;

}

void Motion::Robot::sanityCheck(const unsigned int LookAheadFrames) {
	// timestep
	const float deltaT = (_state->timestamp - _lastTimestamp)/intTimeStampToFloat;

	// remove nans from PID loop
	if (isnan(_w)) {
		_w = 0.0; // stationary is safer for nans

		// reset PID
		_anglePid.clearWindup();
	}

	// predict ahead
	Geometry2d::Point predictedPos = _self->pos +
		_vel * deltaT * LookAheadFrames;

	// avoid opponents
	BOOST_FOREACH(::Robot* r, _state->opp)
	{
		Geometry2d::Circle c(r->pos, Robot_Diameter);

		Geometry2d::Segment s(_self->pos, predictedPos);
		if (r->visible && s.intersects(c))
		{
			Geometry2d::Point dir = r->pos - _self->pos;
			dir = dir.normalized();

			//take off the offending velocity
			_vel -= (dir * dir.dot(_vel));
		}
	}

	//protect against self hit
	BOOST_FOREACH(::Robot* r, _state->self)
	{
		if (r != _self)
		{
			Geometry2d::Circle c(r->pos, Robot_Diameter);

			Geometry2d::Segment s(_self->pos, predictedPos);
			if (r->visible && s.intersects(c))
			{
				Geometry2d::Point dir = r->pos - _self->pos;
				dir = dir.normalized();

				//take off the offending velocity
				_vel -= (dir * dir.dot(_vel));
			}
		}
	}
}

/**
 * This function performs velocity control to convert the path and commands
 * into instantaneous velocity commands that can be converted into wheel
 * commands.
 */
void Motion::Robot::genVelocity(MotionCmd::PathEndType ending)
{
	bool verbose = false;
	if (verbose) cout << "Generating Velocity" << endl;
	//TODO double check the field angle to robot angle conversions!
	//robot space angle = Clipped(team space angle - robot angle)

	const float deltaT = (_state->timestamp - _lastTimestamp)/intTimeStampToFloat;

	// handle facing
	if (_self->cmd.face != MotionCmd::None)
	{
		Geometry2d::Point orientation = _self->cmd.goalOrientation - _self->pos;

		//if the goal position and orientation are the same...use continuous mode
		if (_self->cmd.goalOrientation == _self->cmd.goalPosition)
		{
			_self->cmd.face = MotionCmd::Continuous;
		}

		if (_self->cmd.face == MotionCmd::Endpoint)
		{
			orientation = _self->cmd.goalOrientation - _self->cmd.goalPosition;
		}

		float angleErr = Utils::fixAngleDegrees(orientation.angle() * RadiansToDegrees - _self->angle);

		// avoid singularity when facing exactly opposite of the goal
		float bk_angle_thresh = 2.0;
		if (180.0 - fabs(angleErr) < bk_angle_thresh)
			angleErr = fabs(angleErr);

		// Use pid to determine an angular velocity
		if (verbose) cout << "  angleErr: " << angleErr << endl;
		if (verbose) cout << "  P: " << _anglePid.kp << " I: " << _anglePid.ki << " D: " << _anglePid.kd << endl;
		float targetW = _anglePid.run(angleErr);
		if (verbose) cout << "  w after PID: " << targetW << endl;

		//safety check to avoid singularities
		float angle_thresh = 1e-2;
		if (_self->cmd.goalOrientation.distTo(_self->pos) < angle_thresh)
			targetW = 0;

		/// limit the max rotation based on the full travel path
		/// only if using endpoint mode
		/// in continuous we do not limit
		if (_self->cmd.face == MotionCmd::Endpoint)
		{
			const float fixedLength = _self->cmd.pathLength;

			if (fixedLength > 0)
			{
				//TODO recheck the dynamics calculation
				//the times are too small
				float maxW = fabs(angleErr/_dynamics.travelTime(fixedLength));

				//TODO remove magic number(dynamics travel time is too small)
				maxW /= 10.0f;

				// check saturation of angular velocity
				targetW = Utils::clamp(targetW, maxW, -maxW);
			}
		}

		// clamp w to reasonable bounds to prevent oscillations
		float dW = targetW - _w; // NOTE we do not fix this to range - this is acceleration

		float maxAccel = fabs(_self->config->motion.rotation.acceleration);
		dW = Utils::clamp(dW, maxAccel, -maxAccel);

		_w += dW;
	}
	else
	{
		// No rotation, so set angular velocity to zero
		_w = 0;
	}

	// handle point-to-point driving without pivot
	if (_self->cmd.pivot == MotionCmd::NoPivot)
	{
		//direction of travel
		const Geometry2d::Point dir = _self->cmd.goalPosition - _self->pos;

		// use active braking as necessary
		float dist_stop_thresh = 0.15; // cm
		if (ending == MotionCmd::StopAtEnd && dir.mag() < dist_stop_thresh) {
			stop(deltaT);
			return;
		}

		///basically just a P for target velocity
		// We don't use PID for translation, apparently
		//const float tvel = _posPid.run(length);

		// pull out the robot angle
		const float robotAngle = _self->angle;

		//max velocity info is in the new desired velocity direction
		Dynamics::DynamicsInfo info = _dynamics.info(dir.angle() *
			RadiansToDegrees - robotAngle, _w);

		// bound the velocity by the end of the path if we want to stop at end
		const float vv = sqrtf(2 * _self->cmd.pathLength * info.deceleration);
		if (verbose) cout << "   decel bound: " << info.deceleration << endl;

		// create the maximum velocity given driving until the end of the path
		Geometry2d::Point targetVel = dir.normalized() * vv;
		if (verbose) printPt(targetVel, "   targetVel");

		//last commanded velocity
		Geometry2d::Point dVel = targetVel - _vel;
		if (dVel.isZero())
		{
			// No change needed
			return;
		}
		
		if (verbose) printPt(dVel, "   dVel - raw");

		//!!!acceleration is in the change velocity direction not travel direction
		info = _dynamics.info(dVel.angle() * RadiansToDegrees - robotAngle, _w);
		const float maxAccel = info.acceleration;

		//calc frame acceleration from total acceleration
		const float frameAccel = deltaT * maxAccel;

		//use frame acceleration
		dVel = dVel.normalized() * frameAccel;

		if (verbose) cout << "   frameAccel: " << frameAccel << " maxAccel: " << maxAccel << endl;
		if (verbose) printPt(dVel, "   dVel - saturated");

		// adjust the velocity
		_vel += dVel;
	}
	else /** Handle pivoting */
	{
		Geometry2d::Point dir = _self->cmd.goalOrientation - _self->pos;

		//I know the perpCW is backwards...
		//that is because to create a CW around object velocity I need
		//to rotate the vector the other way
		if (_self->cmd.pivot == MotionCmd::CW)
		{
			dir = dir.perpCCW();
		}
		else
		{
			dir = dir.perpCW();
		}

		dir = dir.normalized();

		_vel = dir * .4;
	}
}

// TEMPORARILY DISABLED
//void Robot::genBezierVelocity() {
//	const bool verbose = false;
//
//	const float deltaT = (_state->timestamp - _lastTimestamp)/intTimeStampToFloat;
//	// error handling
//	size_t degree = _bezierControls.size();
//	if (degree < 2) {
//		stop(deltaT);
//		cout << "Bezier curve of size: " << degree << "!" << endl;
//		return;
//	}
//
//	// generate coefficients
//	vector<float> coeffs;
//	for (size_t i=0; i<degree; ++i) {
//		coeffs.push_back(binomialCoefficient(degree-1, i));
//	}
//
//	// calculate length to allow for determination of time
//	_bezierTotalLength = bezierLength(_bezierControls, coeffs);
//
//	// DEBUG: show the length of the trajectory
//	_state->drawText(QString::number(_bezierTotalLength), Segment(_bezierControls.at(0), _bezierControls.at(1)).center(), Qt::blue);
//
//	// calculate numerical derivative by stepping ahead a fixed constant
//	float lookAheadDist = 0.15; // in meters along path
//	float dt = lookAheadDist/_bezierTotalLength;
//
//	float velGain = 3.0; // FIXME: should be dependent on the length of the curve
//
//	// calculate a target velocity for translation
//	Point targetVel = evaluateBezierVelocity(dt, _bezierControls, coeffs);
//
//	// apply gain
//	targetVel *= velGain;
//
//	// DEBUG: draw the targetVel
//	Segment targetVelLine(pos(), pos() + targetVel);
//	_state->drawLine(targetVelLine, Qt::red);
//	// directly set the velocity
//	_vel = targetVel;
//
//	float targetAngle = 0.0;
//	if (_self->cmd.face == MotionCmd::Continuous) {
//		// look further ahead for angle
//		float WlookAheadDist = 0.30; // in meters along path // prev: 0.15ang_thresh
//		float dtw = WlookAheadDist/_bezierTotalLength;
//		targetAngle = evaluateBezierVelocity(dtw, _bezierControls, coeffs).angle();
//	} else if (_self->cmd.face == MotionCmd::Endpoint) {
//		targetAngle = (_bezierControls.at(degree-1) - _bezierControls.at(degree-2)).angle();
//	}
//
//	// draw the intended facing
//	_state->drawLine(Segment(pos(), pos() + Point::direction(targetAngle).normalized()), Qt::gray);
//
//	// Find the error (degrees)
//	float angleErr = Utils::fixAngleDegrees(targetAngle*RadiansToDegrees - _self->angle);
//
//	// debug GUI for PID commands
//	if (verbose) cout << "Current Kp: " << _anglePid.kp << endl;
//
//	// angular velocity is in degrees/sec
//	_w = _anglePid.run(angleErr);
//}
