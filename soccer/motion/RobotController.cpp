// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <motion/RobotController.hpp>
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

Motion::RobotController::RobotController(OurRobot *robot) :
	_state(robot->state()),
	_self(robot),
	_w(0),
	_velFilter(Point(0.0, 0.0), 1),
	_wFilter(0.0, 1)
{
}

Motion::RobotController::~RobotController()
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
void Motion::RobotController::proc()
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

	_procMutex.unlock();
}

void Motion::RobotController::scaleVelocity() {
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

void Motion::RobotController::sanityCheck(const unsigned int LookAheadFrames) {
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
void Motion::RobotController::genVelocity(MotionCmd::PathEndType ending)
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
		float dist_stop_thresh = 0.05;
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
