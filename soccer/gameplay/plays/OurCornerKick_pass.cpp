#include "OurCornerKick_pass.hpp"
#include <framework/RobotConfig.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include "../ReceivePointEvaluator.hpp"

using namespace std;
using namespace Geometry2d;


static const int requiredBotCount = 2;


REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurCornerKick_Pass, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurCornerKick_Pass)
	}
}

ConfigDouble *Gameplay::Plays::OurCornerKick_Pass::_targetSegmentWidth;

void Gameplay::Plays::OurCornerKick_Pass::createConfiguration(Configuration *cfg)
{
	_targetSegmentWidth = new ConfigDouble(cfg, "OurCornerKick_Pass/Window Width", Field_Length / 4.);
}

Gameplay::Plays::OurCornerKick_Pass::OurCornerKick_Pass(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_receiver1(gameplay),
	_receiver2(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
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


	bool enoughBots = gameplay->playRobots().size() >= 3;


	// return (gs.setupRestart() && gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 1.5)) ? 1 : INFINITY;
	return (enoughBots && gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 2.5)) ? 1 : INFINITY;
}

bool Gameplay::Plays::OurCornerKick_Pass::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	

	if ( available.size() < requiredBotCount ) return false;


	// choose a target for the kick
	// if straight shot on goal is available, take it
	float goal_x = (ball().pos.x < 0) ? Field_GoalWidth / 2.0 : -Field_GoalWidth / 2.0;
	Segment target(Point(goal_x, Field_Length), Point(goal_x, Field_Length - *_targetSegmentWidth));


	if ( !_passDone ) {
		if ( _receiver1.isDone() || _receiver2.isDone() ) {
			_passDone = true;
		}
	}


	//	if the pass isn't done yet, setup for the pass
	if ( !_passDone ) {
		//	Geometry2d::Point FindReceivingPoint(SystemState* state, Robot* robot, Geometry2d::Point ballPos, Geometry2d::Segment receivingLine);

		Segment receiver1Segment(Point(-1.0f, Field_Length - 1.5f), Point(-2, Field_Length - 1.0f));
		Segment receiver2Segment(Point(1.0f, Field_Length - 1.5f), Point(2.0f, Field_Length - 1.0f));




		Point passTarget1;
		Point passTarget2;



		if ( _receiver1.robot ) {
			Point pt = ReceivePointEvaluator::FindReceivingPoint(state(), _receiver1.robot->pos, ball().pos, receiver1Segment);
			passTarget1 = pt;

			state()->drawLine(receiver1Segment.pt[0], receiver1Segment.pt[1], Qt::black);

			// state()->drawCircle(passTarget1, )
			// state()->drawCircle(receivePosition(), Robot_Radius + 0.05, Qt::yellow, QString("DumbReceive"));
		} else {
			passTarget1 = receiver1Segment.center();
		}

		if ( _receiver2.robot ) {
			Point pt = ReceivePointEvaluator::FindReceivingPoint(state(), _receiver2.robot->pos, ball().pos, receiver2Segment);
			passTarget2 = pt;

			state()->drawLine(receiver2Segment.pt[0], receiver2Segment.pt[1], Qt::black);
		} else {
			passTarget2 = receiver2Segment.center();
		}




		Point passerTarget;



		bool firstIsBetter = _receiver1.robot != NULL;	//	FIXME: pick one of the two

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
	}


	assignNearest(_fullback1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_fullback2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));

	_fullback2.run();
	_fullback1.run();
	

	return !_passDone;
}
