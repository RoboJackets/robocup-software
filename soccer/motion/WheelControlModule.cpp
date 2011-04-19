
#include <iostream>

#include <QMutexLocker>

#include <boost/foreach.hpp>
#include <Utils.hpp>
#include <framework/RobotConfig.hpp>
#include <framework/Dynamics.hpp>

#include "WheelControlModule.hpp"

using namespace std;
using namespace Geometry2d;

namespace Motion {

WheelControlModule::WheelControlModule(SystemState *state, Configuration *cfg)
: _state(state), _config(cfg)
{
}

void WheelControlModule::run()
{
	BOOST_FOREACH(OurRobot *robot, _state->self)
	{
		// check that system is valid
		if (robot->visible) {
			// check for direct motion commands
			if (robot->cmd.planner == MotionCmd::DirectMotor) {
				// copy manually specified motor speeds
				size_t j = 0;
				BOOST_FOREACH(const int8_t&  v, robot->cmd.direct_motor_cmds) {
					robot->radioTx.set_motors(j++, v);
				}
			} else if (robot->cmd.planner == MotionCmd::DirectVelocity) {
				// generate commands from manually specified velocities
				genMotor(robot->cmd.direct_trans_vel, robot->cmd.direct_ang_vel, robot);
			} else {
				// convert from generated velocities
				genMotor(robot->cmd_vel, robot->cmd_w, robot);
			}
		}
	}
}

void
WheelControlModule::genMotor(const Geometry2d::Point& vel, float w, OurRobot* robot) {
	_procMutex.lock();
	bool verbose = false;

	// algorithm:
	// 1) saturate the velocities with model bounds (calc current and max)
	// 2) convert to percentage of maximum
	// 3) saturate the percentages
	// 4) apply to wheels
	// 5) flip direction for 2010 robots

	Planning::Dynamics dynamics(robot);

	// angular velocity
	const float maxW = robot->config->motion.rotation.velocity;
	w = Utils::clamp(w, maxW, -maxW);
	if (verbose) cout << "\nCommands: w = " << w << " maxW = " << maxW;

	// handle translational velocity - convert into robot space, then bound
	Point rVel = vel;
	rVel.rotate(Point(), - robot->angle);
	Planning::Dynamics::DynamicsInfo info = dynamics.info(rVel.angle() * RadiansToDegrees, 0);
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
	wPercent = Utils::clamp(wPercent, 1.0, -1.0);

	// Update axles
	Axles axles;
	for (int i = 0; i < 4; ++i)
	{
		Geometry2d::Point axle(robot->config->axles[i]->x, robot->config->axles[i]->y);
		axles[i].wheel = axle.normalized().perpCCW();
	}

	// find the fastest wheel speed, out of all axles with commands
	double vwheelmax = 0.0;
	BOOST_FOREACH(const Axle& axle, axles) {
		float vwheel = fabs(axle.wheel.dot(maxVel));
		if (vwheel > vwheelmax)
			vwheelmax = vwheel;
	}

	// amount of velocity out of max - also signed
	vector<float> vels(4);
	float maxVelPer = 0.0;
	size_t i = 0;
	if (verbose) cout << "  ";
	BOOST_FOREACH(const Axle& axle, axles) {
		float vwheel = axle.wheel.dot(rVel);  // velocity for wheel
		float per = 0.0;
		if (vwheelmax != 0.0)
			per = vwheel/vwheelmax;
		vels[i++] = per;
		if (fabs(per) > maxVelPer)
			maxVelPer = per;

		if (verbose) cout << per << " ";
	}
	if (verbose) cout << endl;

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
		int8_t cmdVel = (int8_t) Utils::clamp(127.0*vel, 126.0, -127.0);
//		if (robot->rev == SystemState::Robot::rev2010) // FIXME: need to enable new 2011 fleet
		if (false)
		{
			robot->radioTx.set_motors(i, -cmdVel);
		} else {
			robot->radioTx.set_motors(i, cmdVel);
		}
		++i;
	}
	if (verbose) cout << endl;
	_procMutex.unlock();
}

}
