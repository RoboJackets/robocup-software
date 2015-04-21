#include <limits>
#include "MyTestPlay.hpp"
#include <iostream>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::MyTestPlay, "Playing")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(MyTestPlay)
	}
}

ConfigDouble *Gameplay::Plays::MyTestPlay::_offense_hysteresis;
ConfigDouble *Gameplay::Plays::MyTestPlay::_support_backoff_thresh;
ConfigDouble *Gameplay::Plays::MyTestPlay::_mark_hysteresis_coeff;
ConfigDouble *Gameplay::Plays::MyTestPlay::_support_avoid_teammate_radius;
ConfigDouble *Gameplay::Plays::MyTestPlay::_support_avoid_shot;
ConfigDouble *Gameplay::Plays::MyTestPlay::_offense_support_ratio;
ConfigDouble *Gameplay::Plays::MyTestPlay::_defense_support_ratio;
ConfigBool   *Gameplay::Plays::MyTestPlay::_use_line_kick;

void Gameplay::Plays::MyTestPlay::createConfiguration(Configuration *cfg)
{
	_offense_hysteresis = new ConfigDouble(cfg, "MyTestPlay/Hystersis Coeff", 0.50);
	_support_backoff_thresh = new ConfigDouble(cfg, "MyTestPlay/Support Backoff Dist", 1.5);
	_mark_hysteresis_coeff = new ConfigDouble(cfg, "MyTestPlay/Mark Hystersis Coeff", 0.9);
	_support_avoid_teammate_radius = new ConfigDouble(cfg, "MyTestPlay/Support Avoid Teammate Dist", 0.5);
	_support_avoid_shot = new ConfigDouble(cfg, "MyTestPlay/Support Avoid Shot", 0.2);
	_offense_support_ratio = new ConfigDouble(cfg, "MyTestPlay/Offense Support Ratio", 0.7);
	_defense_support_ratio = new ConfigDouble(cfg, "MyTestPlay/Defense Support Ratio", 0.9);
	_use_line_kick = new ConfigBool(cfg, "MyTestPlay/Enable Line Kick", true);
}

Gameplay::Plays::MyTestPlay::MyTestPlay(GameplayModule *gameplay):
Play(gameplay),
_leftDefender(gameplay, Behaviors::Defender::Left),
_rightDefender(gameplay, Behaviors::Defender::Right),
_striker(gameplay),
_support1(gameplay),
_support2(gameplay),
_move1(gameplay),
_move2(gameplay),
_move3(gameplay),
_move4(gameplay),
_move5(gameplay)
{

	_leftDefender.otherDefenders.insert(&_rightDefender);
	_rightDefender.otherDefenders.insert(&_leftDefender);
	_letter = 0;
	_startTime = 0;
	// use constant value of mark threshold for now
	_support1.markLineThresh(1.0);
	_support2.markLineThresh(1.0);
}

float Gameplay::Plays::MyTestPlay::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::MyTestPlay::run()
{
	if(_startTime == 0)
		_startTime = timestamp();

	if(timestamp() - _startTime > 3000000)
	{
		_letter++;
		_startTime = timestamp();
		if(_letter == 3)
			_letter = 0;
	}
	set<OurRobot *> available = _gameplay->playRobots();
	resetMoves();
	if(_letter == 0)
	{
		// Make C
		getToPoint(Point(-0.2106,0.99428),_move1, available);
		getToPoint(Point(-1.079,2.13607),_move2, available);
		getToPoint(Point(1.001,2.07889),_move3, available);
		getToPoint(Point(-0.8658,1.3927),_move4, available);
		getToPoint(Point(0.5538,1.30952),_move5, available);
	}
	else if(_letter == 1)
	{
		// Make A
		getToPoint(Point(1.001,1.00802),_move1, available);
		getToPoint(Point(-0.9282,1.7306),_move2, available);
		getToPoint(Point(0.0130001,1.27833),_move3, available);
		getToPoint(Point(1.0114,2.31282),_move4, available);
		getToPoint(Point(0.0182, 2.08409),_move5, available);
	}
	else
	{
		// Make T
		getToPoint(Point(-0.9126,2.33881),_move5, available);
		getToPoint(Point(-0.2834,1.73579),_move1, available);
		getToPoint(Point(-0.9282,1.7306),_move2, available);
		getToPoint(Point(-0.9178,1.08599),_move3, available);
		getToPoint(Point(0.3458,1.7306),_move4, available);
	}
	return true;
}


void Gameplay::Plays::MyTestPlay::getToPoint(Geometry2d::Point p, Behaviors::Move b, set<OurRobot*> &robots)
{

    b.target = p;
    b.face = Point(0.0,0.0);
	assignNearest(b.robot, robots, p);
    b.run();
}

void Gameplay::Plays::MyTestPlay::resetMoves()
{
	_move1 = Behaviors::Move(_gameplay);
	_move2 = Behaviors::Move(_gameplay);
	_move3 = Behaviors::Move(_gameplay);
	_move4 = Behaviors::Move(_gameplay);
	_move5 = Behaviors::Move(_gameplay);
}

void Gameplay::Plays::MyTestPlay::waitFor(int secs)
{
	uint64_t startTime = timestamp();
	while((timestamp() - startTime) < (secs * 1000000));
	cout << "3 Seconds!" << endl;
}
