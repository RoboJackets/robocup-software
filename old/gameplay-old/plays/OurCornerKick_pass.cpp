#include "OurCornerKick_pass.hpp"
#include <RobotConfig.hpp>

#include <iostream>
#include "../evaluation/ReceivePointEvaluator.hpp"

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
ConfigDouble *Gameplay::Plays::OurCornerKick_Pass::_receiverChoiceHysterisis;

void Gameplay::Plays::OurCornerKick_Pass::createConfiguration(Configuration *cfg)
{
	_targetSegmentWidth = new ConfigDouble(cfg, "OurCornerKick_Pass/Window Width", Field_Length / 4.);
	_receiverChoiceHysterisis = new ConfigDouble(cfg, "OurCornerKick_Pass/Receiver Choice Hysterisis", 0);
}

Gameplay::Plays::OurCornerKick_Pass::OurCornerKick_Pass(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_defender1(gameplay, Behaviors::Defender::Left),
	_defender2(gameplay, Behaviors::Defender::Right),
	_receiver2(gameplay),
	_pdt(gameplay, &_passer),
	_receiver1(gameplay),
	_passCtxt(gameplay, &_passer)
{
	// _center1.target = Point(0.0, Field_Length /2.0);

	//	FIXME: setup pass ctxt

	_passCtxt.addReceiver(&_receiver1);
	_passCtxt.addReceiver(&_receiver2);

	_firstRun = true;
	_choosinessTimeout = 1000 * 5;
}

float Gameplay::Plays::OurCornerKick_Pass::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	Point ballPos = gameplay->state()->ball.pos;
	bool chipper_available = false;
	for (OurRobot * r :  gameplay->playRobots())
	{
		if (r && r->chipper_available())
		{
			chipper_available = true;
			break;
		}
	}


	chipper_available = true;	//	FIXME: hack


	bool enoughBots = gameplay->playRobots().size() >= 2;


	// return (gs.setupRestart() && gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 1.5)) ? 1 : INFINITY;
	return (enoughBots && (gs.ourDirect() || gs.ourIndirect()) && chipper_available && ballPos.y > (Field_Length - 2.5)) ? 1 : INFINITY;
}

bool Gameplay::Plays::OurCornerKick_Pass::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	//	start a timer as soon as the play starts ACTUALLY running
	GameState::State gpState = _gameplay->state()->gameState.state;
	if ( _firstRun && (gpState == GameState::Ready || gpState == GameState::Playing) ) {
		_startTime = timestamp();
		_firstRun = false;
	}


	if ( available.size() < requiredBotCount ) return false;


	//	if the pass isn't done yet, setup for the pass
	if ( !_passCtxt.done() ) {
		//	Geometry2d::Point FindReceivingPoint(SystemState* state, Robot* robot, Geometry2d::Point ballPos, Geometry2d::Segment receivingLine);

		Segment receiver1Segment(Point(-0.3f, Field_Length - 1.7f), Point(-1.6f, Field_Length - 0.8f));
		Segment receiver2Segment(Point(0.5f, Field_Length - 1.5f), Point(1.5f, Field_Length - 1.0f));

		//	we've timed out!  stop thinking and start doing
		bool needToGetOnTheBus = timestamp() - _startTime > _choosinessTimeout;

		if ( !needToGetOnTheBus ) {
			_passCtxt.chooseReceivePointForReceiverAlongSegment(&_receiver1, receiver1Segment);
			_passCtxt.chooseReceivePointForReceiverAlongSegment(&_receiver2, receiver2Segment);
		}


		assignNearest(_passer.robot, available, ball().pos);
		assignNearest(_receiver1.robot, available, _receiver1.actionTarget);
		assignNearest(_receiver2.robot, available, _receiver2.actionTarget);

		_passCtxt.run();
	}



	assignNearest(_defender1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_defender2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));

	_defender2.run();
	_defender1.run();
	

	return !_passCtxt.done();
}
