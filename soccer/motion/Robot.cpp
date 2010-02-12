// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <iostream>
#include <string>
#include "Robot.hpp"
#include <QMutexLocker>
#include <Geometry2d/Point.hpp>
#include <Team.h>
#include <boost/foreach.hpp>
#include <Utils.hpp>
#include <Constants.hpp>

#include "Pid.hpp"
#include "framework/Module.hpp"

using namespace std;
using namespace Geometry2d;
using namespace Motion;
using namespace Packet;


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

/** prints out a labeled point */
void printPt(const Geometry2d::Point& pt, const string& s="") {
	cout << s << ": (" << pt.x << ", " << pt.y << ")" << endl;
}

/** Constant for timestamp to seconds? */
const float intTimeStampToFloat = 1000000.0f;

Robot::Robot(const ConfigFile::MotionModule::Robot& cfg, unsigned int id) :
	_id(id), _isConfigLoaded(false)
{
	_state = 0;
	_self = 0;
	
	_planner.setDynamics(&_dynamics);
	_planner.maxIterations(250);
	

	//since radio currently only handles 4 motors
	//don't make axles (and thus don't drive) if not 4 configured
	BOOST_FOREACH(const Point& p, cfg.axles)
	{
		_axles.append(Axle(p.normalized()));
	}
	
	_lastTimestamp = Utils::timestamp();
	
	_calibState = InitCalib;
}

Robot::~Robot()
{
}

void Robot::setPosKp(double value)
{
	_posPid.kp = value;
	//_config.pos.p = value;
}

void Robot::setPosKi(double value)
{
	_posPid.ki = value;
	//_config.pos.i = value;
}

void Robot::setPosKd(double value)
{
	_posPid.kd = value;
	//_config.pos.d = value;
}

void Robot::setAngKp(double value)
{
	_anglePid.kp = value;
	//_config.angle.p = value;
}

void Robot::setAngKi(double value)
{
	_anglePid.ki = value;
	//_config.angle.i = value;
}

void Robot::setAngKd(double value)
{
	_anglePid.kd = value;
	//_config.angle.d = value;
}

void Robot::setSystemState(SystemState* state)
{
	_state = state;
	_self = &_state->self[_id];
}

void Robot::proc()
{
	_procMutex.lock();
	// Check to make sure the system is valid
	if (_self && _self->valid)
	{
		// get the dynamics from the config
		if (!_isConfigLoaded) {
			_dynamics.setConfig(_self->config.motion); //FIXME - change setConfig call cycle
			_isConfigLoaded = true;
		}
		
		// set the correct PID parameters for position and angle
		_posPid.kp = _self->config.motion.pos.p;
		_posPid.ki = _self->config.motion.pos.i;
		_posPid.kd = _self->config.motion.pos.d;
		
		_anglePid.kp = _self->config.motion.angle.p;
		_anglePid.ki = _self->config.motion.angle.i;
		_anglePid.kd = _self->config.motion.angle.d;
		
		if (_state->gameState.state == GameState::Halt)
		{
			// don't do anything if we aren't transmitting to this robot
			_self->radioTx.valid = false;
		}
		else
		{
#if 1
			// switch between planner types
			switch(_self->cmd.planner) {
			// handle direct velocity commands (NOT IMPLEMENTED)
			case Packet::MotionCmd::DirectVelocity:
			{
				// set the velocities from the gameplay module
				_vel = _self->cmd.direct_trans_vel;
				_w = _self->cmd.direct_ang_vel;

				// TODO: make this smarter, as it should be an actual
				// velocity controller that handles the current velocity as well
				break;
			}
			// handle explicit path generation (short-circuit RRT)
			case Packet::MotionCmd::Path:
			{
				// create a new path
				Planning::Path path;

				// set the precomputed path
				path.points = _self->cmd.explicitPath;

				// assign the path
				_path = path;

				// create the velocities
				genVelocity(_self->cmd.pathEnd);
				break;
			}

			// handle time-position control
			case Packet::MotionCmd::TimePosition:
			{
				// generate the velocity command for time-position control
				genTimePosVelocity();
				break;
			}

			// default RRT-based planner that also handles pivoting
			case Packet::MotionCmd::RRT:
			{
				if (_self->cmd.pivot == MotionCmd::NoPivot)
				{
					//new path if better than old
					Planning::Path path;

					// determine the obstacles
					ObstacleGroup& og = _self->obstacles;

					// run the RRT planner to generate a new plan
					_planner.run(_self->pos, _self->angle, _self->vel, _self->cmd.goalPosition, &og, path);
					_path = path;

					// create the velocities
					genVelocity(_self->cmd.pathEnd);
				}
				else // handle pivot
				{
					//clear old path for when we don't want to pivot
					_path.clear();

					//TODO fixme...

					//for now look at the ball
					_self->cmd.face = Packet::MotionCmd::Continuous;
					_self->cmd.goalOrientation = _self->cmd.pivotPoint;

					// create the velocities
					genVelocity(_self->cmd.pathEnd);
				}
				break;
			}
			}

			// generate motor outputs based on velocity
			genMotor();
#else
			calib();
#endif
			
			_self->radioTx.valid = true;
			_self->radioTx.board_id = _self->shell;
		}
	}
	_lastTimestamp = _state->timestamp;
	
	_procMutex.unlock();
}

void Robot::slow()
{
}

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
}


void Robot::stop() {
	// handle rotation with PID - force value to zero
	_w = _anglePid.run(_w);

	// get a model to use for approximating robot movement
	const float robotAngle = _self->angle;
	Dynamics::DynamicsInfo info = _dynamics.info(_vel.angle()*RadiansToDegrees - robotAngle, _w);

	// find magnitude of the maximum deceleration
	const float vv = info.deceleration;

	// find the magnitude of the current velocity
	const float vcur = _vel.mag();

	// if we can go straight to zero, do it, otherwise drop by maximum deceleration
	if (vcur < vv)
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

	// handle saturating the maximum velocity
	if (_vel.mag() > maxVel)
	{
		_vel = _vel.normalized() * maxVel;
	}
}

void Robot::sanityCheck(const unsigned int LookAheadFrames) {
	// timestep
	const float deltaT = (_state->timestamp - _lastTimestamp)/intTimeStampToFloat;

	// predict ahead
	Geometry2d::Point predictedPos = _self->pos +
		_vel * deltaT * LookAheadFrames;

	// avoid opponents
	BOOST_FOREACH(Packet::LogFrame::Robot& r, _state->opp)
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
	BOOST_FOREACH(Packet::LogFrame::Robot& r, _state->self)
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
void Robot::genVelocity(Packet::MotionCmd::PathEndType ending)
{
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
		
		const float angleErr = Utils::fixAngleDegrees(orientation.angle() * RadiansToDegrees - _self->angle);
		
		// Use pid to determine an angular velocity
		_w = _anglePid.run(angleErr);
		
		//safety check to avoid singularities
		float angle_thresh = 1e-2;
		if (_self->cmd.goalOrientation.distTo(_self->pos) < angle_thresh)
		{
			_w = 0;
		}
		
		/// limit the max rotation based on the full travel path
		/// only if using endpoint mode
		/// in continuous we do not limit
		if (_self->cmd.face == MotionCmd::Endpoint)
		{
			const float fixedLength = _planner.fixedPathLength();
			
			if (fixedLength > 0)
			{
				//TODO recheck the dynamics calculation
				//the times are too small
				float maxW = fabs(angleErr/_dynamics.travelTime(fixedLength));
				
				//TODO remove magic number(dynamics travel time is too small)
				maxW /= 10.0f;
				
				// check saturation of angular velocity
				_w = saturate(_w, maxW, -maxW);
			}
		}
	}
	else
	{
		// No rotation, so set angular velocity to zero
		_w = 0;
	}
	
	// handle point-to-point driving without pivot
	if (_self->cmd.pivot == Packet::MotionCmd::NoPivot)
	{
		//dynamics path
		float length = _path.length();
		
		// handle direct point commands where the length may be very small
		if (fabs(length) < 1e-5) {
			length = _self->pos.distTo(_path.points[0]);
		}

		//target point is the last point on the closest segment
		Geometry2d::Point targetPos = _path.points[0]; // first point
		
		if (_path.points.size() > 1)
		{
			targetPos = _path.points[1];
		}
		
		//ideally we want to travel towards 2 points ahead...due to delays
		if (_path.points.size() > 2)
		{
			targetPos = _path.points[2];
		}
		
		//direction of travel
		const Geometry2d::Point dir = targetPos - _self->pos;

		///basically just a P for target velocity
		// We don't use PID for translation, apparently
		//const float tvel = _posPid.run(length);

		// pull out the robot angle
		const float robotAngle = _self->angle;
		
		//max velocity info is in the new desired velocity direction
		Dynamics::DynamicsInfo info = _dynamics.info(dir.angle() * 
			RadiansToDegrees - robotAngle, _w);
		
		// find magnitude of the velocity
		const float vv = sqrtf(2 * length * info.deceleration);
		
		// create the velocity vector
		Geometry2d::Point targetVel = dir.normalized() * vv;
		
		//last commanded velocity
		Geometry2d::Point dVel = targetVel - _vel;
		
		//!!!acceleration is in the change velocity direction not travel direction
		info = _dynamics.info(dVel.angle() * RadiansToDegrees - robotAngle, _w);
		const float maxAccel = info.acceleration;
		
		//calc frame acceleration from total acceleration
		const float frameAccel = deltaT * maxAccel;
		
		//use frame acceleration
		dVel = dVel.normalized() * frameAccel;
		
		// adjust the velocity
		_vel += dVel;
		
		// scale velocity due to commands
		scaleVelocity();

	}
	else /** Handle pivoting */
	{

		Geometry2d::Point dir = _self->cmd.goalOrientation - _self->pos;
		
		//I know the perpCW is backwards...
		//that is because to create a CW around object velocity I need
		//to rotate the vector the other way
		if (_self->cmd.pivot == Packet::MotionCmd::CW)
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
	
	// sanity check the velocities due to obstacles
	const unsigned int LookAheadFrames = 5;
	sanityCheck(LookAheadFrames);
}

/** print out a time-pos node for debugging */
void printTimePos(const MotionCmd::PathNode& node) {
	cout << "PathNode: "
		 << "   Angle: " << node.rot << endl;
	printPt(node.pos, "   Pos: ");
	cout << "   Time: " << node.time << endl;
}

/** Print out a path of path-time nodes */
void printTimePosPath(const vector<MotionCmd::PathNode>& path, const string& s="") {
	cout << "TimePos Path [" << s << "] (Size " << path.size() << ")" << endl;
	BOOST_FOREACH(MotionCmd::PathNode node, path) {
		printTimePos(node);
	}
	cout << endl;
}

void Robot::genTimePosVelocity()
{
	bool verbose = false;

	// find the time stepsize
	const float deltaT = (_state->timestamp - _lastTimestamp)/intTimeStampToFloat;

	// determine what the change in time since last frame was
	float cur_time = (_state->timestamp-_self->cmd.start_time)/intTimeStampToFloat;
	if (verbose) cout << "Current time: " << cur_time << endl;

	// extract a portion of the path
	vector<MotionCmd::PathNode>& full_path = _self->cmd.timePosPath;
	//if (verbose) printTimePosPath(full_path, "Full path");

	// we only want a specific number of path nodes in advance, and after the current time
	int lookahead = 3;
	vector<MotionCmd::PathNode> path;
	MotionCmd::PathNode node;
	int i = 0;
	BOOST_FOREACH(node, full_path)
	{
		// look for a node that is in the future
		if (node.time < cur_time)
			continue;

		// if we have a node in the future, keep counting until reached the lookahead count
		if (i < lookahead) {
			path.push_back(node);
			++i;
		}
	}

	// handle no nodes in the future
	if (path.size() == 0)
	{
		// nowhere to go, so we stop the robot
		stop();
		return;
	}
	if (verbose) printTimePosPath(path, "Future Path");
	if (verbose) printPt(_self->pos, "Current Pos");

	// handle facing - use PID to get the closest rotational position
	float targetAngle = path[0].rot;
	float angleErr = Utils::fixAngleDegrees(targetAngle - _self->angle);
	_w = _anglePid.run(angleErr);

	// handle translation - use simple method for driving towards goal
	// drive straight at point as fast as possible
	Geometry2d::Point target = path[0].pos;
	Geometry2d::Point dir = target - _self->pos;

	// pull out the robot angle
	const float robotAngle = _self->angle;

	//max velocity info is in the new desired velocity direction
	Dynamics::DynamicsInfo info = _dynamics.info(dir.angle() *
		RadiansToDegrees - robotAngle, _w);

	// find the length of the path
	float length = dir.mag();

	// find magnitude of the velocity
	const float vv = sqrtf(2 * length * info.deceleration);
	if (verbose) cout << "Deceleration: " << info.deceleration << endl;

	// don't use maximum velocity for travelTime estimate, adjust down to account
	// for acceleration
	// this should be a function of the distance - longer distances get closer to one
	float adjust = 0.8;

	// determine how long it will take at the target velocity
	if (verbose) cout << "length: " << length << endl;
	if (verbose) cout << "vv: " << vv << endl;
	//float travelTime = length/vv;
	float travelTime = length/(vv*adjust);
	if (verbose) cout << "Estimated travel time: " << travelTime << endl;

	// determine how long we want it to take
	float targetTime = path[0].time - cur_time;

	// weight the velocity vector to arrive at a particular time
	float weight = 1.0;
	if (travelTime < targetTime) {
		weight = travelTime/targetTime; // bring down speed to match the time requirement
	}

	// create the velocity vector
	Geometry2d::Point targetVel = dir.normalized() * vv * weight;

	//last commanded velocity
	Geometry2d::Point dVel = targetVel - _vel;

	//!!!acceleration is in the change velocity direction not travel direction
	info = _dynamics.info(dVel.angle() * RadiansToDegrees - robotAngle, _w);
	const float maxAccel = info.acceleration;

	//calc frame acceleration from total acceleration
	const float frameAccel = deltaT * maxAccel;

	//use frame acceleration
	dVel = dVel.normalized() * frameAccel;

	// adjust the velocity
	_vel += dVel;

	// scale velocities as per commands
	scaleVelocity();

	// sanity check the velocities due to obstacles
	const unsigned int LookAheadFrames = 5;
	sanityCheck(LookAheadFrames);

	if (verbose) cout << "At end of genTimePosVelocity()" << endl;
}

void Robot::genMotor(bool old)
{
	if (!old)
	{
		// handle saturation of angular velocity
		float w =  _w;
		const float maxW = _self->config.motion.rotation.velocity;
		w = saturate(w, maxW, -maxW);

		//amount of rotation out of max
		//[-1...1]
		float wPercent = 0;
		
		if (maxW != 0)
		{
			wPercent = w/maxW;
		}
		
		if (wPercent > 1)
		{
			wPercent = 1;
		}
		
		int8_t rotSpeed = 127 *  wPercent;
		
		for (unsigned int i=0 ; i<4; ++i)
		{
			_self->radioTx.motors[i] = rotSpeed;
		}
		
		if (rotSpeed < 0)
		{
			rotSpeed = -rotSpeed;
		}
		
		//max linear speed remaining
		int8_t maxSpeed = 127 - rotSpeed;
		
		Geometry2d::Point rVel = _vel;
		rVel.rotate(Point(), -_self->angle);
		
		Dynamics::DynamicsInfo info = _dynamics.info(rVel.angle() * RadiansToDegrees, 0);
		
		//vmax of the robot in desired velocity direction
		const float vm = info.velocity;
		
		//limit max velocity in that direction
		if (rVel.mag() > vm)
		{
			rVel = rVel.normalized() * vm;
		}
		
		Geometry2d::Point vmax = rVel.normalized() * vm;
		
		float max = 0;
		BOOST_FOREACH(Robot::Axle& axle, _axles)
		{
			const float vwheel = fabs(axle.wheel.dot(vmax));
			if (vwheel > max)
			{
				max = vwheel;
			}
		}
		
		int i=0;
		BOOST_FOREACH(Robot::Axle& axle, _axles)
		{
			if (i >= 4)
			{
				printf("Radio packet does not support more than 4 wheels!\n");
				break;
			}
			
			//max ground velocity of the wheel
			const float vwheel = axle.wheel.dot(rVel);
			float per = vwheel/max;
			
			//this really won't happen because rVel has been limited to the right number
			if (per > 1)
			{
				per = 1;
			}
			
			_self->radioTx.motors[i] += int8_t(maxSpeed * per);
			i++;
		}
	}
	else
	{
		/// old style generation....
		genMotorOld();
	}
}

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
		_self->radioTx.motors[j++] = (int8_t) a.motor;

		//radio does not support more than 4 wheels
		if (j >= 4)
		{
			break;
		}
	}
}

void Robot::calib()
{
	const float fieldW2 = Constants::Field::Width/2.0f;

	int debugRobotId = 9;

	if (_calibState == InitCalib)
	{
		_calibInfo = CalibInfo();

		float y = 1.5;

		_calibInfo.startPos = Geometry2d::Point(fieldW2 - .3, y);
		_calibInfo.endPos = Geometry2d::Point(0, y);

		_calibState = InitialPoint;
	}
	else if (_calibState == InitialPoint)
	{
		Geometry2d::Point dir = (_calibInfo.endPos - _calibInfo.startPos).normalized();

		_self->cmd.goalOrientation = _calibInfo.endPos + dir * 1.0;
		_self->cmd.face = Packet::MotionCmd::Endpoint;
		_self->cmd.vScale = .5;

		Planning::Path path;
		ObstacleGroup& og = _self->obstacles;
		_planner.run(_self->pos, _self->angle, _self->vel, _calibInfo.startPos, &og, path);
		_path = path;

		float destAngle =  dir.angle() * RadiansToDegrees;

		float a = fabs(Utils::fixAngleDegrees(_self->angle - destAngle));

		if (_calibInfo.startPos.nearPoint(_self->pos, .1) && a < 5)
		{
			_calibInfo.startTime = Utils::timestamp();
			_calibState = Wait1;
		}

		genVelocity(_self->cmd.pathEnd);
		genMotor();
	}
	else if (_calibState == Wait1)
	{
		const float dur = (_state->timestamp - _calibInfo.startTime)/1000000.0f;
		if (dur > 3)
		{
			_calibInfo.startTime = Utils::timestamp();
			_calibInfo.lastPos = _self->pos;
			_calibInfo.pSum = 0;
			_calibInfo.startPos = _self->pos;

			_calibState = Travel;
			_calibInfo.outSpeed = _calibInfo.speed;

			if (_self->shell == debugRobotId)
			{
				printf("Speed: %d\n", _calibInfo.outSpeed);
			}
		}
	}
	else if (_calibState == Travel)
	{
		float duration = (_state->timestamp-_calibInfo.startTime)/1000000.0f;

		Geometry2d::Point deltaP = (_self->pos - _calibInfo.lastPos);

		//add to running point sum
		_calibInfo.pSum += deltaP.mag();

		//print duration, velocty mag, del position mag
		if (_self->shell == debugRobotId)
		{
			printf("calib %f %f %f\n", duration, _self->vel.mag(), deltaP.mag());
			fflush(stdout);
		}

		//end
		if (_self->pos.y >= Constants::Field::Length - .2 || _self->pos.y <= .2 ||
			_self->pos.x <= (-Constants::Field::Width/2.0 + .2) ||
			_self->pos.x >= (Constants::Field::Width/2.0 - .2) ||
			duration > 5 || !_self->valid)
		{
			//goto deccel
			_calibState = Decel;
		}

		_calibInfo.lastPos = _self->pos;

		int8_t wspeed = _calibInfo.outSpeed;

#if 1
		_self->radioTx.motors[0] = -wspeed;
		_self->radioTx.motors[1] = wspeed;
		_self->radioTx.motors[2] = wspeed;
		_self->radioTx.motors[3] = -wspeed;
#else //45
		_self->radioTx.motors[0] = 0;
		_self->radioTx.motors[1] = wspeed;
		_self->radioTx.motors[2] = 0;
		_self->radioTx.motors[3] = -wspeed;
#endif
	}

	if (_calibState == Decel)
	{
		_calibInfo.outSpeed -= 30;

		if (_calibInfo.outSpeed < 0)
		{
			_calibInfo.outSpeed = 0;
		}

		if (!_self->valid)
		{
			_calibInfo.outSpeed = 0;
		}

		_self->radioTx.motors[0] = -_calibInfo.outSpeed;
		_self->radioTx.motors[1] = _calibInfo.outSpeed;
		_self->radioTx.motors[2] = _calibInfo.outSpeed;
		_self->radioTx.motors[3] = -_calibInfo.outSpeed;

		if (_calibInfo.outSpeed == 0)
		{
			_calibState = InitialPoint;

			//if we ran at the max speed
			//we are all done
			if (_calibInfo.speed == 127)
			{
				//end
				_calibState = End;
			}

			//increment speed by fixed amount
			_calibInfo.speed += 10;

			//otherwise go to max speed
			if (_calibInfo.speed > 127)
			{
				_calibInfo.speed = 127;
			}
		}
	}
}
