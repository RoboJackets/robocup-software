#include "OurGoalKick_Pass.hpp"
#include <RobotConfig.hpp>

#include <iostream>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurGoalKick_Pass, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurGoalKick_Pass)
	}
}

ConfigDouble *Gameplay::Plays::OurGoalKick_Pass::_downFieldRange;
ConfigInt *Gameplay::Plays::OurGoalKick_Pass::_kicker_power;
ConfigDouble *Gameplay::Plays::OurGoalKick_Pass::_field_length_mult;
ConfigDouble *Gameplay::Plays::OurGoalKick_Pass::_field_width_mult;
ConfigDouble *Gameplay::Plays::OurGoalKick_Pass::_field_length_off;
ConfigDouble *Gameplay::Plays::OurGoalKick_Pass::_field_width_off;

void Gameplay::Plays::OurGoalKick_Pass::createConfiguration(Configuration *cfg)
{
	_kicker_power = new ConfigInt(cfg, "OurGoalKick_Pass/Kicker Power", 255);
	_downFieldRange = new ConfigDouble(cfg, "OurGoalKick_Pass/Downfield Range", Field_Length / 2.);
	_field_length_mult = new ConfigDouble(cfg, "OurGoalKick_Pass/Centers Length Multiplier", 0.6);
	_field_length_off = new ConfigDouble(cfg, "OurGoalKick_Pass/Centers Length Offset", 0.0);
	_field_width_mult = new ConfigDouble(cfg, "OurGoalKick_Pass/Centers Width Multiplier", 0.15);
	_field_width_off = new ConfigDouble(cfg, "OurGoalKick_Pass/Centers Width Offset", 0.0);
}

Gameplay::Plays::OurGoalKick_Pass::OurGoalKick_Pass(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_receiver1(gameplay),
	_receiver2(gameplay),
	_defender1(gameplay, Behaviors::Defender::Left),
	_defender2(gameplay, Behaviors::Defender::Right),
	_passCtxt(_gameplay, &_passer)
{
	_defender2.otherDefenders.insert(&_defender1);
	_defender1.otherDefenders.insert(&_defender2);

	_passCtxt.addReceiver(&_receiver1);
	_passCtxt.addReceiver(&_receiver2);
}

float Gameplay::Plays::OurGoalKick_Pass::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	Point ballPos = gameplay->state()->ball.pos;

	bool ballAtOurEnd = ballPos.y < 1.5;

	int cmp = gameplay->playName().compare( QString("OurGoalKick_Pass") );
	bool currentlyRunning = cmp == 0;

	return (gs.ourDirect() && (ballAtOurEnd || currentlyRunning) ) ? 1 : INFINITY;;
}

bool Gameplay::Plays::OurGoalKick_Pass::run()
{
	set<OurRobot *> available = _gameplay->playRobots();


	if ( !_passCtxt.done() ) {
		Point rcv1Target = Point(
				-Field_Width * _field_width_mult->value() + _field_width_off->value(),
				Field_Length * _field_length_mult->value() + _field_length_off->value()
				);

		Point rcv2Target = Point(
				Field_Width * _field_width_mult->value() + _field_width_off->value(),
				Field_Length * _field_length_mult->value() + _field_length_off->value()
				);


		_passCtxt.setReceivePointForReceiver(&_receiver1, rcv1Target);
		_passCtxt.setReceivePointForReceiver(&_receiver2, rcv2Target);



		_passCtxt.updateReceiverScoresBasedOnChannelWidth();

		assignNearest(_passer.robot, available, ball().pos);
		assignNearest(_receiver1.robot, available, _receiver1.actionTarget);
		assignNearest(_receiver2.robot, available, _receiver2.actionTarget);

		_passCtxt.run();
	}

	assignNearest(_defender1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_defender2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));


	_defender1.run();
	_defender2.run();
	

	return !_passCtxt.done();
}
