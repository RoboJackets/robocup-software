#include "Robot.hpp"
#include "Ball.hpp"
#include "Environment.hpp"

#include <Utils.hpp>
#include <Constants.hpp>

#include <boost/foreach.hpp>

#define ROLLER  1
#define KICKER  0
#define CHIPPER 0

using namespace Geometry2d;

Robot::Robot(Environment* env, unsigned int id,  Robot::RobotRevision rev, const Geometry2d::Point& pos) :
			Entity(env), shell(id), _rev(rev), _pos(pos), _lastKicked(0)
{
	visibility = 100;
	ballSensorWorks = true;
	chargerWorks = true;

	// temp state info
	_theta = 0.0;
	_omega = 0.0;

}

Robot::~Robot()
{
}

void Robot::initRoller()
{
}

void Robot::initKicker()
{
}

void Robot::initWheels()
{
}

void Robot::position(float x, float y)
{
	_pos = Geometry2d::Point(x,y);
}

void Robot::velocity(float x, float y, float w)
{
	_vel = Geometry2d::Point(x,y);
	_omega = 0.0;
}

float Robot::getAngle() const
{
	return _theta;
}

void Robot::radioTx(const Packet::RadioTx::Robot *data)
{
//	Geometry2d::Point axles[4] =
//	{
//			Point( .08,  .08),
//			Point( .08, -.08),
//			Point(-.08, -.08),
//			Point(-.08,  .08)
//	};

	// FIXME: interface for motors changed - now uses body velocities, rather than motor commands
//	for (unsigned int i = 0 ; i < 4 ; ++i)
//	{
//		Point wheel(-axles[i].y, axles[i].x);
//		wheel = wheel.normalized();
//		wheel.rotate(Point(), getAngle());
//
//		float target = data->motors(i) / 127.0f * 1.2;
//
//		// reverse for 2011 robots
//		if (_rev == rev2011)
//		{
//			target = -target;
//		}
//
//		// TODO: apply force
//	}

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
