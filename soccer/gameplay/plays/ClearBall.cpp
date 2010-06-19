#include "ClearBall.hpp"
#include <boost/foreach.hpp>

using namespace std;

// TODO: CHANGE APPLICABLE TO USE CORRECT DEFENSE CALC.
// TODO: ASSIGN KICKER TO BE ONETOUCH

Gameplay::Plays::ClearBall::ClearBall(GameplayModule *gameplay):
	Play(gameplay, 4),
	_kicker(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_kicker1(gameplay),
	_kicker2(gameplay),
	_oppDistMin(1.5),
	_selfDistMax(0.75)
{
}

bool Gameplay::Plays::ClearBall::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	bool gameplayApplicable = true; //_gameplay->state()->stateID.posession == Packet::LogFrame::DEFENSE; // is this correct?

	return refApplicable && gameplayApplicable;
}

bool Gameplay::Plays::ClearBall::assign(set<Robot *> &available)
{
	_kicker.assign(available);
	_fullback1.assign(available);
	_kicker1.assign(available);
	_kicker2.assign(available);

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::ClearBall::run()
{
	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::Kick::Done)
		_kicker.restart();

	// run the kick play
	_kicker.run();
	return true;
}

float Gameplay::Plays::ClearBall::score()
{
	float selfBallDistMin = 999;
	float oppBallDistMin = 999;
	Geometry2d::Point ballPos = _gameplay->state()->ball.pos;

	// calculate closest (non-goalie) self robot to ball
	Robot* goalie = (_gameplay->goalie() ? _gameplay->goalie()->robot() : (Robot*)0);
	BOOST_FOREACH(Robot *r, _gameplay->self){
		float ballDist = ballPos.distTo(r->pos());
		if(r!=goalie && selfBallDistMin > ballDist){
			selfBallDistMin = ballDist;
		}
	}

	// calculate closest opp to ball
	BOOST_FOREACH(Robot *r, _gameplay->opp){
		float ballDist = ballPos.distTo(r->pos());
		if(oppBallDistMin > ballDist){
			oppBallDistMin = ballDist;
		}
	}

	if(selfBallDistMin < _selfDistMax && oppBallDistMin > _oppDistMin){
		return 0.0;
	}else{
		return 999;
	}
}
