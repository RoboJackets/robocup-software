// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <iostream>
#include <cmath>
#include <boost/foreach.hpp>
#include <Constants.hpp>
#include "StateIDModule.hpp"

using namespace std;
using namespace StateIdentification;
using namespace Packet;
using namespace Utils;

StateIDModule::StateIDModule(SystemState *state) : Module("State ID")
{
	_state = state;
}

StateIDModule::~StateIDModule()
{
	
}

void StateIDModule::run()
{
	// get the state from the state variable
	LogFrame::GameStateID& stateID = _state->stateID;

	// determine possession
	stateID.posession = updatePossession(stateID.posession);

	// determine field position
	stateID.field_pos = updateFieldPos(stateID.field_pos);

	// write back an updated state
	_state->stateID = stateID;

}

LogFrame::Possession StateIDModule::updatePossession(const LogFrame::Possession& cur_state) {

	// handle restart cases
	bool restart = _state->gameState.setupRestart();
	if (restart && _state->gameState.ourRestart)
	{
		return LogFrame::OFFENSE;
	} else if (restart && !_state->gameState.ourRestart)
	{
		return LogFrame::DEFENSE;
	}

	// thresholds
	float angle_thresh = 20 * DegreesToRadians;
	float dist_thresh = Constants::Robot::Radius + Constants::Ball::Radius + 0.1;
	float speed_thresh = 1.0;

	// get ball state
	Geometry2d::Point ball_pos = _state->ball.pos;
	Geometry2d::Point ball_vel = _state->ball.vel;

	bool opp_has_ball = false;
	bool self_has_ball = false;

	// check for other team's possession
	BOOST_FOREACH(LogFrame::Robot robot, _state->opp) {
		//opp has ball if it is close to a robot and moving in the same direction
		double dist = ball_pos.distTo(robot.pos);
		float vel_angle_diff = abs(fixAngleRadians(ball_vel.angle() - robot.vel.angle()));
		bool ball_downfield = ball_pos.y < robot.pos.y - Constants::Robot::Radius;
		bool isSameSpeed = abs(robot.vel.mag() - ball_vel.mag()) < speed_thresh;

		if (dist < dist_thresh &&
				vel_angle_diff < angle_thresh &&
				ball_downfield &&
				isSameSpeed)
		{
			opp_has_ball = true;
			break;
		}
	}

	// check if we have ball
	BOOST_FOREACH(LogFrame::Robot robot, _state->self) {
		//opp has ball if it is close to a robot and moving in the same direction
		double dist = ball_pos.distTo(robot.pos);
		float vel_angle_diff = abs(fixAngleRadians(ball_vel.angle() - robot.vel.angle()));
		bool ball_downfield = ball_pos.y < robot.pos.y - Constants::Robot::Radius;
		bool isSameSpeed = abs(robot.vel.mag() - ball_vel.mag()) < speed_thresh;

		if (dist < dist_thresh &&
				vel_angle_diff < angle_thresh &&
				ball_downfield &&
				isSameSpeed)
		{
			self_has_ball = true;
			break;
		}
	}


	// handle dynamic cases
	switch (cur_state) {
		case LogFrame::OFFENSE:
			// switch out of offense only when the other team gets the ball
			if (opp_has_ball) return LogFrame::DEFENSE;
			break;
		case LogFrame::DEFENSE:
			// switch out of defense only when our team gets the ball
			if (self_has_ball) return LogFrame::OFFENSE;
			break;
		case LogFrame::FREEBALL:
			// switch to offense or defense when a team gets the ball
			if (self_has_ball && !opp_has_ball)
				return LogFrame::OFFENSE;
			else if (!self_has_ball && opp_has_ball)
				return LogFrame::DEFENSE;
			// if neither team has it, or both are close enough to have it,
			// stay free ball
			else if (!(self_has_ball xor opp_has_ball))
				return cur_state;
	};

	return cur_state;
}

LogFrame::BallFieldPos StateIDModule::updateFieldPos(const LogFrame::BallFieldPos& cur_pos) {

	// get ball state
	Geometry2d::Point ball_pos = _state->ball.pos;
	Geometry2d::Point ball_vel = _state->ball.vel;

	//TODO: Add hystersis using the projected ball location

	//Set field position predicates
	if (ball_pos.y < Constants::Field::Length /3)
	{
		return LogFrame::HOMEFIELD;
	}
	else if (ball_pos.y < Constants::Field::Length * 2/3 &&
			ball_pos.y > Constants::Field::Length /3)
	{
		return LogFrame::MIDFIELD;
	}
	else
	{
		return LogFrame::OPPFIELD;
	}
}
