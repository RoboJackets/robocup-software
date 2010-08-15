#include "ClearBall.hpp"
#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY(Gameplay::Plays::ClearBall)

static const float oppDistMin = 1.5; // minimum distance opp must be for score calc
static const float selfDistMax = 0.75; // maximum distance self robot must be for score calc

Gameplay::Plays::ClearBall::ClearBall(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_kicker1(gameplay),
	_kicker2(gameplay),
	_done(false)
{
	set<Robot *> available = gameplay->robots();
	
	// may want to comment this out
	_kicker.aimType(Behaviors::Kick::ONETOUCH);
	
	cout << "assigning clearball play" << std::endl;
	_done = false;

	float selfBallDistMin = 999;
	Geometry2d::Point ballPos = _gameplay->state()->ball.pos;
	Robot* closest = 0;

	// calculate closest (non-goalie) self robot to ball
	Robot* goalie = (_gameplay->goalie() ? _gameplay->goalie()->robot() : (Robot*)0);
	BOOST_FOREACH(Robot *r, _gameplay->self){
		float ballDist = ballPos.distTo(r->pos());
		if(r!=goalie && selfBallDistMin > ballDist){
			selfBallDistMin = ballDist;
			closest = r;
		}
	}

	if(!closest){return;}

	_kicker.assignOne(closest);
	available.erase(closest);
	_fullback1.assign(available);
	_kicker1.assign(available);
	_kicker2.assign(available);

	_robots.insert(_kicker.robot());
	_robots.insert(_fullback1.robot());
	_robots.insert(_kicker1.robot());
	_robots.insert(_kicker2.robot());
}

bool Gameplay::Plays::ClearBall::run()
{
	// check if the robot is in done state
	if (_kicker.getState() == Gameplay::Behaviors::Kick::Done)
		_done = true;
	if (ball().pos.x > Constants::Field::Length/2)
		_done = true;

	// run the kick play
	_kicker.run();

	// run the other plays
	_fullback1.run();
	_kicker1.run();
	_kicker2.run();

	return _done;
}

float Gameplay::Plays::ClearBall::scoreRobot(Robot *r)
{
	return _gameplay->state()->ball.pos.distTo(r->pos());
}

float Gameplay::Plays::ClearBall::score(GameplayModule *gameplay)
{
	bool refApplicable =gameplay->state()->gameState.playing();
	bool gameplayApplicable = true && gameplay->state()->stateID.posession == SystemState::DEFENSE;

	if (!(refApplicable && gameplayApplicable && gameplay->robots().size() >= 4))
	{
		return INFINITY;
	}

	float selfBallDistMin = 999;
	float oppBallDistMin = 999;
	Geometry2d::Point ballPos = gameplay->state()->ball.pos;

	// calculate closest (non-goalie) self robot to ball
	Robot* goalie = (gameplay->goalie() ? gameplay->goalie()->robot() : (Robot*)0);
	BOOST_FOREACH(Robot *r, gameplay->self){
		float ballDist = ballPos.distTo(r->pos());
		if(r!=goalie && selfBallDistMin > ballDist){
			selfBallDistMin = ballDist;
		}
	}

	// calculate closest opp to ball
	BOOST_FOREACH(Robot *r, gameplay->opp){
		float ballDist = ballPos.distTo(r->pos());
		if(oppBallDistMin > ballDist){
			oppBallDistMin = ballDist;
		}
	}

	if(selfBallDistMin < selfDistMax && oppBallDistMin > oppDistMin)
	{
		return 0.0;
	}else{
		return 999;
	}
}
