#include "Robot.hpp"
#include "Ball.hpp"
#include "Environment.hpp"

#include <Utils.hpp>
#include <Constants.hpp>

#include <Geometry2d/TransformMatrix.hpp>

#include <boost/foreach.hpp>

using namespace Geometry2d;

Robot::Robot(Environment* env, unsigned int id,  Robot::RobotRevision rev, const Geometry2d::Point& startPos) :
			Entity(env), shell(id), _rev(rev), _startPos(startPos), _lastKicked(0)
{
	visibility = 100;
	ballSensorWorks = true;
	chargerWorks = true;
}

Robot::~Robot()
{
}

void Robot::initPhysics(const bool& blue)
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

void Robot::position(float x, float y){ }

void Robot::velocity(float x, float y, float z){ }

Geometry2d::Point Robot::getPosition() const{
	return _startPos;
}

float Robot::getAngle() const{
	return 0;
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
