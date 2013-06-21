#include "OurCornerKick_pass.hpp"
#include <framework/RobotConfig.hpp>
#include <boost/foreach.hpp>
#include <iostream>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurCornerKick_Pass, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurCornerKick_Pass)
	}
}

ConfigDouble *Gameplay::Plays::OurCornerKick_Pass::_targetSegmentWidth;
ConfigDouble *Gameplay::Plays::OurCornerKick_Pass::_minChipRange;
ConfigDouble *Gameplay::Plays::OurCornerKick_Pass::_maxChipRange;
ConfigInt *Gameplay::Plays::OurCornerKick_Pass::_chipper_power;

void Gameplay::Plays::OurCornerKick_Pass::createConfiguration(Configuration *cfg)
{
	_chipper_power = new ConfigInt(cfg, "OurCornerKick_Pass/Chipper Power", 127);
	_targetSegmentWidth = new ConfigDouble(cfg, "OurCornerKick_Pass/Window Width", Field_Length / 4.);
	_minChipRange = new ConfigDouble(cfg, "OurCornerKick_Pass/Min Chip Range", 0.3);
	_maxChipRange = new ConfigDouble(cfg, "OurCornerKick_Pass/Max Chip Range", 3.0);
}

Gameplay::Plays::OurCornerKick_Pass::OurCornerKick_Pass(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_receiver1(gameplay),
	_receiver2(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_pivotKicker(gameplay),
	_pdt(gameplay, &_passer)
{
	// _center1.target = Point(0.0, Field_Length /2.0);

	_passDone = false;
}

float Gameplay::Plays::OurCornerKick_Pass::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	Point ballPos = gameplay->state()->ball.pos;
	bool chipper_available = false;
	BOOST_FOREACH(OurRobot * r, gameplay->playRobots())
	{
		if (r && r->chipper_available())
		{
			chipper_available = true;
			break;
		}
	}


	chipper_available = true;	//	FIXME: hack


	// return (gs.setupRestart() && gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 1.5)) ? 1 : INFINITY;
	return (gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 1.5)) ? 1 : INFINITY;
}

bool Gameplay::Plays::OurCornerKick_Pass::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	// pick the chip kicker - want to chip most of the time
	// if (!assignNearestChipper(_kicker.robot, available, ball().pos))
	// {
	// 	return false;
	// }

	// assignNearest(_center1.robot, available, Point(-0.5, Field_Length));
	// assignNearest(_center2.robot, available, Point(0.5, Field_Length));

	// choose a target for the kick
	// if straight shot on goal is available, take it
	float goal_x = (ball().pos.x < 0) ? Field_GoalWidth / 2.0 : -Field_GoalWidth / 2.0;
	Segment target(Point(goal_x, Field_Length), Point(goal_x, Field_Length - *_targetSegmentWidth));

	// camp first robot to receive "pass"
	// _center1.target.x = (ball().pos.x < 0) ? Field_GoalWidth / 2.0 + 0.5 : -(Field_GoalWidth / 2.0 + 0.5);
	// _center1.target.y = Field_Length - *_targetSegmentWidth / 2.0;

	// camp second side for TODO: short "pass"
	// _center2.target.x = (ball().pos.x < 0) ? -(Field_GoalWidth / 2.0 + 0.5) : Field_GoalWidth / 2.0 + 0.5 ;
	// _center2.target.y = Field_Length - *_targetSegmentWidth / 2.0;

	// setup kicker from parameters - want to use chipper when possible
	// _kicker.setTarget(target);
	// _kicker.use_chipper = true;
	// _kicker.use_line_kick = true;
	// _kicker.calculateChipPower(_center1.target.distTo(ball().pos));
	// _kicker.kick_power = *_chipper_power;
	// _kicker.minChipRange = *_minChipRange;
	// _kicker.maxChipRange = *_maxChipRange;


	if ( !_passDone ) {
		if ( _receiver1.isDone() || _receiver2.isDone() ) {
			_passDone = true;
		}
	}




	//	if the pass isn't done yet, setup for the pass
	if ( !_passDone ) {
		Point passTarget1(-1.0f, Field_Length - 1.0f);	//	semi-arbitrarily-chosen points
		Point passTarget2(1.0f, Field_Length - 1.0f);	//



		Point passerTarget;	//	FIXME: pick one of the two



		bool firstIsBetter = false;

		//	setup passer && receivers appropriately for the chosen point
		if ( firstIsBetter ) {
			passerTarget = passTarget1;

			_passer.partner = &_receiver1;
			_receiver1.partner = &_passer;

			_receiver2.partner = NULL;
			if ( _receiver2.robot ) _receiver2.robot->addText("Dummy");
		} else {
			passerTarget = passTarget2;

			_passer.partner = &_receiver2;
			_receiver2.partner = &_passer;

			_receiver1.partner = NULL;
		}


		_passer.actionTarget = passerTarget;

		_receiver1.actionTarget = passTarget1;
		_receiver2.actionTarget = passTarget2;


		assignNearest(_passer.robot, available, ball().pos);
		assignNearest(_receiver1.robot, available, passTarget1);
		assignNearest(_receiver2.robot, available, passTarget2);



		_pdt.backoff.robots.clear();
		_pdt.backoff.robots.insert(_passer.robot);
		
		//	run passing behaviors
		_pdt.run();	//	runs passer
		_receiver1.run();
		_receiver2.run();

		uint8_t dspeed = 60;
		if ( _receiver1.robot ) _receiver1.robot->dribble(dspeed);
		if ( _receiver2.robot ) _receiver2.robot->dribble(dspeed);
	} else {


		_pdt.backoff.robots.clear();



		_passer.robot = NULL;
		_receiver1.robot = NULL;




		//	FIXME: setup pivot kicker

		assignNearest(_pivotKicker.robot, available, ball().pos);

		_pivotKicker.run();
	}


	assignNearest(_fullback1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_fullback2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));

	_fullback2.run();
	_fullback1.run();
	

	// if ( !_pdt.keepRunning() ) {
	// 	cout << "OurCornerKick_Pass double touched! - play will terminate" << endl;
	// }

	// return _pdt.keepRunning();
	return true;
}
