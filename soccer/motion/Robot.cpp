// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <motion/Robot.hpp>
#include <QMutexLocker>
#include <Geometry2d/Point.hpp>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>
#include <Utils.hpp>
#include <Constants.hpp>

#include <motion/Pid.hpp>

using namespace std;
using namespace boost::assign;
using namespace Geometry2d;
using namespace Motion;
using namespace Planning;

/** prints out a labeled point */
void printPt(const Geometry2d::Point& pt, const string& s="") {
	cout << s << ": (" << pt.x << ", " << pt.y << ")" << endl;
}

/** Handles saturation of a bounded value */
float saturate(float value, float max, float min) {
	if (value > max)
	{
		return max;
	}
	else if (value < min)
	{
		return min;
	}
	return value;
}

/** Constant for timestamp to seconds */
const float intTimeStampToFloat = 1000000.0f;

Robot::Robot(const ConfigFile::MotionModule::Robot& cfg, unsigned int id) :
	_id(id), _velFilter(Point(0.0, 0.0), 1), _wFilter(0.0, 1)
{
	_state = 0;
	_self = 0;
	_w = 0;

	//since radio currently only handles 4 motors
	//don't make axles (and thus don't drive) if not 4 configured
	BOOST_FOREACH(const Point& p, cfg.axles)
	{
		_axles.append(Axle(p.normalized()));
	}

	_lastTimestamp = Utils::timestamp();

//	_calibState = InitCalib;
}

Robot::~Robot()
{
}

void Robot::setAngKp(double value)
{
	_procMutex.lock();
	_anglePid.kp = value;
	_procMutex.unlock();
}

void Robot::setAngKi(double value)
{
	_procMutex.lock();
	_anglePid.ki = value;
	_procMutex.unlock();
}

void Robot::setAngKd(double value)
{
	_procMutex.lock();
	_anglePid.kd = value;
	_procMutex.unlock();
}

void Robot::setSystemState(SystemState* state)
{
	_state = state;
	_self = &_state->self[_id];
}


/**
 * Phases in motor commands:
 * 	I:   Create a plan (RRT)
 *  II:  Create velocities
 *  III: Sanity Check velocities (obstacle avoidance, bounding)
 *  IV:  Smooth Velocities (Filter system)
 *  V:   Convert to motor commands
 */
void Robot::proc()
{
	bool verbose = false;
	_procMutex.lock();
	// Check to make sure the system is valid
	if (_self && _self->valid)
	{
		// get the dynamics from the config
		_dynamics.setConfig(_self->config.motion);

		// set the correct PID parameters for angle
		_anglePid.kp = _self->config.motion.angle.p;
		_anglePid.ki = _self->config.motion.angle.i;
		_anglePid.kd = _self->config.motion.angle.d;

		// set coefficients for the output filters
		_velFilter.setCoeffs(_self->config.motion.output_coeffs);
		_wFilter.setCoeffs(_self->config.motion.output_coeffs);

		if (_state->gameState.state == GameState::Halt)
		{
			// don't do anything if we aren't transmitting to this robot
			for (int i = 0; i < 4; ++i)
			{
				_self->radioTx->set_motors(i, 0);
			}
		}
		else
		{
			// record the type of planner for future use
			_plannerType = _self->cmd.planner;		// get the dynamics from the config
			_dynamics.setConfig(_self->config.motion);

			// set the correct PID parameters for angle
			_anglePid.kp = _self->config.motion.angle.p;
			_anglePid.ki = _self->config.motion.angle.i;
			_anglePid.kd = _self->config.motion.angle.d;

			// set coefficients for the output filters
			_velFilter.setCoeffs(_self->config.motion.output_coeffs);
			_wFilter.setCoeffs(_self->config.motion.output_coeffs);

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
			// handle direct motor commands
			case MotionCmd::DirectMotor:
			{
				// FIXME: we really should do nothing here, and pass to WheelController

				// short circuit controller completely
//				size_t i = 0;
//				BOOST_FOREACH(const int8_t& vel, _self->cmd.direct_motor_cmds) {
//					_self->radioTx->set_motors(i++, vel);
//				}

				break;
			}

			// Catch force stop commands
			case MotionCmd::ForceStop:
			{
				const float deltaT = (_state->timestamp - _lastTimestamp)/intTimeStampToFloat;
				stop(deltaT);
			}

			// default RRT-based planner that also handles pivoting
			case MotionCmd::Point:
			{
				if (_self->cmd.pivot == MotionCmd::NoPivot)
				{

					// FIXME: no need for path planning here
//					//new path if better than old
//					Planning::Path newPath;
//
//					// determine the obstacles
//					ObstacleGroup& og = _self->obstacles;
//
//					// run the RRT planner to generate a new plan
//					_planner.run(_self->pos, _self->angle, _self->vel, _self->cmd.goalPosition, &og, newPath);
//
//					_path = newPath;

					// create the velocities
					genVelocity(_self->cmd.pathEnd);
				}
				else // handle pivot
				{
					//clear old path for when we don't want to pivot
//					_path.clear();

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

			// generate motor outputs based on velocity
			if (!_useOldMotorGen) {
				genMotor();
			} else {
				genMotorOld();
			}

			// save the commanded velocities to packet for inspection in tree
			_self->cmd_vel = _vel;
			_self->cmd_w = _w;

			// FIXME: we set the radio messages in WheelController
//			_self->radioTx->set_board_id(_self->shell);
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

void Robot::stop(float dtime) {
	// handle rotation with PID - force value to zero
	_w = _anglePid.run(_w);

	// force to zero if close
	float ang_thresh = 0.2;
	if (_w < ang_thresh)
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

void Robot::scaleVelocity() {
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
	_self->cmd.vScale = saturate(_self->cmd.vScale, 1.0, 0.0);

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

void Robot::sanityCheck(const unsigned int LookAheadFrames) {
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
	BOOST_FOREACH(SystemState::Robot& r, _state->opp)
	{
		Geometry2d::Circle c(r.pos, Constants::Robot::Diameter);

		Geometry2d::Segment s(_self->pos, predictedPos);
		if (r.valid && s.intersects(c))
		{
			Geometry2d::Point dir = r.pos - _self->pos;
			dir = dir.normalized();

			//take off the offending velocity
			_vel -= (dir * dir.dot(_vel));
		}
	}

	//protect against self hit
	BOOST_FOREACH(SystemState::Robot& r, _state->self)
	{
		if (r.shell != _self->shell)
		{
			Geometry2d::Circle c(r.pos, Constants::Robot::Diameter);

			Geometry2d::Segment s(_self->pos, predictedPos);
			if (r.valid && s.intersects(c))
			{
				Geometry2d::Point dir = r.pos - _self->pos;
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
void Robot::genVelocity(MotionCmd::PathEndType ending)
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
				targetW = saturate(targetW, maxW, -maxW);
			}
		}

		// clamp w to reasonable bounds to prevent oscillations
		float dW = targetW - _w; // NOTE we do not fix this to range - this is acceleration

		float maxAccel = fabs(_self->config.motion.rotation.acceleration);
		dW = saturate(dW, maxAccel, -maxAccel);

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
//		if (_path.points.empty())
//		{
//			// No path: stop.
//			//FIXME - This should never happen.  We must keep the check, but fix the cause.
//			printf("genVelocity: empty path, stopping\n");
//			_vel = Point();
//			return;
//		}
//
//		//dynamics path
//		float length = _path.length();
//
//		// handle direct point commands where the length may be very small
//		if (fabs(length) < 1e-5) {
//			length = _self->pos.distTo(_path.points[0]);
//		}
//
//		//target point is the last point on the closest segment
//		Geometry2d::Point targetPos = _path.points[0]; // first point
//
//		if (_path.points.size() > 1)
//		{
//			targetPos = _path.points[1];
//		}
//
//		//ideally we want to travel towards 2 points ahead...due to delays
//		if (_path.points.size() > 2)
//		{
//			targetPos = _path.points[2];
//		}

		// ************************** SPLIT HERE (PointController) ********************** //

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

void Robot::genMotor() {
	bool verbose = false;

	// algorithm:
	// 1) saturate the velocities with model bounds (calc current and max)
	// 2) convert to percentage of maximum
	// 3) saturate the percentages
	// 4) apply to wheels
	// 5) flip direction for 2010 robots

	// angular velocity
	float w =  _w;
	const float maxW = _self->config.motion.rotation.velocity;
	w = saturate(w, maxW, -maxW);
	if (verbose) cout << "\nCommands: w = " << w << " maxW = " << maxW;

	// handle translational velocity - convert into robot space, then bound
	Point rVel = _vel;
	rVel.rotate(Point(), -_self->angle);
	Dynamics::DynamicsInfo info = _dynamics.info(rVel.angle() * RadiansToDegrees, 0);
	const float maxSpeed = info.velocity;
	const Point maxVel = rVel.normalized() * maxSpeed;
	rVel = Point::saturate(rVel, maxSpeed);
	if (verbose) cout << "  rVel: (" << rVel.x << ", " << rVel.y << ")" <<
			             "  maxVel: (" << maxVel.x << ", " << maxVel.y << ")" << endl;

	// amount of rotation out of max - note these are signed
	float wPercent = 0.0;
	if (maxW != 0.0)
	{
		wPercent = w/maxW;
	}
	wPercent = saturate(wPercent, 1.0, -1.0);

	// find the fastest wheel speed, out of all axles with commands
	double vwheelmax = 0.0;
	BOOST_FOREACH(Robot::Axle& axle, _axles) {
		float vwheel = fabs(axle.wheel.dot(maxVel));
		if (vwheel > vwheelmax)
			vwheelmax = vwheel;
	}

	// amount of velocity out of max - also signed
	vector<float> vels(4);
	float maxVelPer = 0.0;
	size_t i = 0;
	BOOST_FOREACH(Robot::Axle& axle, _axles) {
		float vwheel = axle.wheel.dot(rVel);  // velocity for wheel
		float per = 0.0;
		if (vwheelmax != 0.0)
			per = vwheel/vwheelmax;
		vels[i++] = per;
		if (fabs(per) > maxVelPer)
			maxVelPer = per;
	}

	// mix the control inputs together
	vector<float> wheelVels(4); // signed percents of maximum from each wheel
	i = 0;
	BOOST_FOREACH(float& vel, wheelVels) {
		vel = wPercent + (1.0-fabs(wPercent)) * vels[i++];
	}

	// convert to integer commands and assign
	i = 0;
	if (verbose) cout << "Motor percent at assign: ";
	BOOST_FOREACH(const float& vel, wheelVels) {
		if (verbose) cout << " " << vel;
		int8_t cmdVel = (int8_t) saturate(127.0*vel, 126.0, -127.0);
		if (_self->rev == SystemState::Robot::rev2008) {
			_self->radioTx->set_motors(i++, cmdVel);
		} else if (_self->rev == SystemState::Robot::rev2010) {
			_self->radioTx->set_motors(i++, -cmdVel);
		}
	}
	if (verbose) cout << endl;
}

//void Robot::genMotor()
//{
//	// algorithm:
//	// 1) assign rotational velocities to wheels
//	// 2) determine remaining amount of rotational velocity left, and use for translation
//	// 3) assign to wheels
//
//	// handle saturation of angular velocity
//	float w =  _w;
//	const float maxW = _self->config.motion.rotation.velocity;
//	w = saturate(w, maxW, -maxW);
//
//	//amount of rotation out of max
//	//[-1...1]
//	float wPercent = 0.0;
//
//	if (maxW != 0)
//	{
//		wPercent = w/maxW;
//	}
//	wPercent = saturate(wPercent, 1.0, -1.0);
//
//	// assign rotational velocities to the wheels
//	int8_t rotVel = 127 *  wPercent;
//	for (unsigned int i=0 ; i<4; ++i)
//	{
//		_self->radioTx->motors[i] = rotVel;
//	}
//
//	// determine the speed
//	int8_t rotSpeed = abs(rotVel);
//
//	//max linear speed remaining
//	int8_t maxSpeed = 127 - rotSpeed;
//
//	// handle translational velocity - convert into robot space
//	Geometry2d::Point rVel = _vel;
//	rVel.rotate(Point(), -_self->angle);
//
//	Dynamics::DynamicsInfo info = _dynamics.info(rVel.angle() * RadiansToDegrees, 0);
//
//	//vmax of the robot in desired velocity direction
//	const float vm = info.velocity;
//
//	//limit max velocity in that direction
//	if (rVel.mag() > vm)
//	{
//		rVel = rVel.normalized() * vm;
//	}
//
//	// find max velocity
//	Geometry2d::Point vmax = rVel.normalized() * vm;
//
//	float max = 0;
//	BOOST_FOREACH(Robot::Axle& axle, _axles)
//	{
//		const float vwheel = fabs(axle.wheel.dot(vmax));
//		if (vwheel > max)
//		{
//			max = vwheel;
//		}
//	}
//
//	int i=0;
//	BOOST_FOREACH(Robot::Axle& axle, _axles)
//	{
//		if (i >= 4)
//		{
//			printf("Radio packet does not support more than 4 wheels!\n");
//			break;
//		}
//
//		//max ground velocity of the wheel
//		const float vwheel = axle.wheel.dot(rVel);
//		float per = vwheel/max;
//
//		//this really won't happen because rVel has been limited to the right number
//		per = saturate(per, 1.0, -1.0);
//
//		_self->radioTx->motors[i] += int8_t(maxSpeed * per);
//		i++;
//	}
//
//}

void Robot::genMotorOld() {
	//convert the velocity command into robot space
	Geometry2d::Point rVel = _vel;
	rVel.rotate(Point(), -_self->angle);

	float maxGenWheelVel = 0;

	BOOST_FOREACH(Robot::Axle& axle, _axles)
	{
		axle.motor = axle.wheel.dot(rVel);
		axle.motor += _w;

		if (abs(axle.motor > maxGenWheelVel))
		{
			maxGenWheelVel = axle.motor;
		}

		axle.lastWheelVel = axle.motor;
	}

	int j = 0;
	const float _maxWheelVel = 255;
	BOOST_FOREACH(Robot::Axle& a, _axles)
	{
		//one of the wheels saturated...scale others back accordingly
		if (maxGenWheelVel > _maxWheelVel)
		{
			a.motor *= _maxWheelVel / maxGenWheelVel;
		}

		//set outgoing motor
		_self->radioTx->set_motors(j++, (int8_t) a.motor);

		//radio does not support more than 4 wheels
		if (j >= 4)
		{
			break;
		}
	}
}

//void Robot::calib()
//{
//	const float fieldW2 = Constants::Field::Width/2.0f;
//
//	int debugRobotId = 9;
//
//	if (_calibState == InitCalib)
//	{
//		_calibInfo = CalibInfo();
//
//		float y = 1.5;
//
//		_calibInfo.startPos = Geometry2d::Point(fieldW2 - .3, y);
//		_calibInfo.endPos = Geometry2d::Point(0, y);
//
//		_calibState = InitialPoint;
//	}
//	else if (_calibState == InitialPoint)
//	{
//		Geometry2d::Point dir = (_calibInfo.endPos - _calibInfo.startPos).normalized();
//
//		_self->cmd.goalOrientation = _calibInfo.endPos + dir * 1.0;
//		_self->cmd.face = MotionCmd::Endpoint;
//		_self->cmd.vScale = .5;
//
//		Planning::Path path;
//		ObstacleGroup& og = _self->obstacles;
//		_planner.run(_self->pos, _self->angle, _self->vel, _calibInfo.startPos, &og, path);
//		_path = path;
//
//		float destAngle =  dir.angle() * RadiansToDegrees;
//
//		float a = fabs(Utils::fixAngleDegrees(_self->angle - destAngle));
//
//		if (_calibInfo.startPos.nearPoint(_self->pos, .1) && a < 5)
//		{
//			_calibInfo.startTime = Utils::timestamp();
//			_calibState = Wait1;
//		}
//
//		genVelocity(_self->cmd.pathEnd);
//		genMotor();
//	}
//	else if (_calibState == Wait1)
//	{
//		const float dur = (_state->timestamp - _calibInfo.startTime)/1000000.0f;
//		if (dur > 3)
//		{
//			_calibInfo.startTime = Utils::timestamp();
//			_calibInfo.lastPos = _self->pos;
//			_calibInfo.pSum = 0;
//			_calibInfo.startPos = _self->pos;
//
//			_calibState = Travel;
//			_calibInfo.outSpeed = _calibInfo.speed;
//
//			if (_self->shell == debugRobotId)
//			{
//				printf("Speed: %d\n", _calibInfo.outSpeed);
//			}
//		}
//	}
//	else if (_calibState == Travel)
//	{
//		float duration = (_state->timestamp-_calibInfo.startTime)/1000000.0f;
//
//		Geometry2d::Point deltaP = (_self->pos - _calibInfo.lastPos);
//
//		//add to running point sum
//		_calibInfo.pSum += deltaP.mag();
//
//		//print duration, velocty mag, del position mag
//		if (_self->shell == debugRobotId)
//		{
//			printf("calib %f %f %f\n", duration, _self->vel.mag(), deltaP.mag());
//			fflush(stdout);
//		}
//
//		//end
//		if (_self->pos.y >= Constants::Field::Length - .2 || _self->pos.y <= .2 ||
//			_self->pos.x <= (-Constants::Field::Width/2.0 + .2) ||
//			_self->pos.x >= (Constants::Field::Width/2.0 - .2) ||
//			duration > 5 || !_self->valid)
//		{
//			//goto deccel
//			_calibState = Decel;
//		}
//
//		_calibInfo.lastPos = _self->pos;
//
//		int8_t wspeed = _calibInfo.outSpeed;
//
//#if 1
//		_self->radioTx->set_motors(0, -wspeed);
//		_self->radioTx->set_motors(1, wspeed);
//		_self->radioTx->set_motors(2, wspeed);
//		_self->radioTx->set_motors(3, -wspeed);
//#else //45
//		_self->radioTx->motors[0] = 0;
//		_self->radioTx->motors[1] = wspeed;
//		_self->radioTx->motors[2] = 0;
//		_self->radioTx->motors[3] = -wspeed;
//#endif
//	}
//
//	if (_calibState == Decel)
//	{
//		_calibInfo.outSpeed -= 30;
//
//		if (_calibInfo.outSpeed < 0)
//		{
//			_calibInfo.outSpeed = 0;
//		}
//
//		if (!_self->valid)
//		{
//			_calibInfo.outSpeed = 0;
//		}
//
//		_self->radioTx->set_motors(0, -_calibInfo.outSpeed);
//		_self->radioTx->set_motors(1, _calibInfo.outSpeed);
//		_self->radioTx->set_motors(2, _calibInfo.outSpeed);
//		_self->radioTx->set_motors(3, -_calibInfo.outSpeed);
//
//		if (_calibInfo.outSpeed == 0)
//		{
//			_calibState = InitialPoint;
//
//			//if we ran at the max speed
//			//we are all done
//			if (_calibInfo.speed == 127)
//			{
//				//end
//				_calibState = End;
//			}
//
//			//increment speed by fixed amount
//			_calibInfo.speed += 10;
//
//			//otherwise go to max speed
//			if (_calibInfo.speed > 127)
//			{
//				_calibInfo.speed = 127;
//			}
//		}
//	}
//}
