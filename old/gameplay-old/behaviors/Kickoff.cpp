#include "Kickoff.hpp"
#include <algorithm>
#include <RobotConfig.hpp>

using namespace std;
using namespace Geometry2d;

namespace Gameplay {
namespace Behaviors {
REGISTER_CONFIGURABLE(Kickoff)
}
}

ConfigBool *Gameplay::Behaviors::Kickoff::_enableChip;
ConfigDouble *Gameplay::Behaviors::Kickoff::_chipMinRange;
ConfigDouble *Gameplay::Behaviors::Kickoff::_chipMaxRange;

void Gameplay::Behaviors::Kickoff::createConfiguration(Configuration *cfg)
{
	_enableChip = new ConfigBool(cfg, "Kickoff/Enable Chipping", false);
	_chipMinRange = new ConfigDouble(cfg, "Kickoff/Chip Min Range", 0.3);
	_chipMaxRange = new ConfigDouble(cfg, "Kickoff/Chip Max Range", 2.0);
}

Gameplay::Behaviors::Kickoff::Kickoff(GameplayModule *gameplay):
SingleRobotBehavior(gameplay),
_kick(gameplay),
_bump(gameplay)
{
	mode = Mode_None;

	// default is kick
	useRandomKick = true;
	kickPower = 255;
	enableChip = true;
	enableBump = false;

	// seed random number generator
	srand(state()->timestamp);
}

void Gameplay::Behaviors::Kickoff::chooseMode() {
	mode = Mode_Chip;
//	if (useRandomKick)
//	{
//		// add enabled modes to pool and pick one at random
//		vector<KickoffMode> modes;
//
//		if (robot->kickerWorks() && *robot->status->kicker_enabled)
//		{
//			modes.push_back(Mode_Kick);
//			modes.push_back(Mode_KickRightCorner);
//			modes.push_back(Mode_KickLeftCorner);
//		}
//
//		if (enableChip && *_enableChip && robot->hardwareVersion() == Packet::RJ2011 && robot->kickerWorks() && *robot->status->chipper_enabled)
//		{
//			modes.push_back(Mode_Chip);
//			modes.push_back(Mode_ChipRightCorner);
//			modes.push_back(Mode_ChipLeftCorner);
//		}
//
//		// handle case where only option is bump
//		if (modes.empty())
//		{
//			mode = Mode_Bump;
//		} else
//		{
//			random_shuffle(modes.begin(), modes.end());
//			mode = modes.front();
//		}
//
//	} else
//	{
//		// if not random, pick first on list that is available
//		mode = (robot->kicker_available()) ? Mode_Kick : Mode_Bump;
//
//		if (enableChip && *_enableChip)
//		{
//			mode = Mode_Chip;
//		}
//	}
}

void Gameplay::Behaviors::Kickoff::executeMode() {

	const Geometry2d::Segment leftTarget(
					Geometry2d::Point(-Field_Width / 2.0, Field_Length),
					Geometry2d::Point(-1.0, Field_Length));
	const Geometry2d::Segment rightTarget(
					Geometry2d::Point(Field_Width / 2.0, Field_Length),
					Geometry2d::Point(1.0, Field_Length));

	switch (mode) {
	case Mode_Kick:
		_kick.use_chipper = false;
		_kick.setTargetGoal();
		_kick.run();
		break;
	case Mode_KickLeftCorner:
		_kick.setTarget(leftTarget);
		_kick.use_chipper = false;
		_kick.run();
		break;
	case Mode_KickRightCorner:
		_kick.setTarget(rightTarget);
		_kick.use_chipper = false;
		_kick.run();
		break;
	case Mode_Chip:
		_kick.use_chipper = true;
		_kick.forceChip = true;
		_kick.setTargetGoal();
		_kick.maxChipRange = *_chipMaxRange;
		_kick.minChipRange = *_chipMinRange;
		_kick.run();
		break;
	case Mode_ChipRightCorner:
		_kick.use_chipper = true;
		_kick.forceChip = true;
		_kick.setTarget(rightTarget);
		_kick.maxChipRange = *_chipMaxRange;
		_kick.minChipRange = *_chipMinRange;
		_kick.run();
		break;
	case Mode_ChipLeftCorner:
		_kick.use_chipper = true;
		_kick.forceChip = true;
		_kick.setTarget(rightTarget);
		_kick.maxChipRange = *_chipMaxRange;
		_kick.minChipRange = *_chipMinRange;
		_kick.run();
		break;
	case Mode_Bump:
		_bump.target = Point(0.0, Field_Length);
		_bump.run();
		break;
	case Mode_None:
		break;
	}
}

bool Gameplay::Behaviors::Kickoff::run()
{
	if (!robot || !robot->visible)
	{
		return false;
	}

	_kick.robot = robot;
	_bump.robot = robot;

	_kick.kick_power = kickPower;
	_kick.use_line_kick = true;

	// Use the real ball position if we have it.  Otherwise, assume the middle of the field.
	Geometry2d::Point ballPos(0, Field_Length / 2);
	if (ball().valid)
	{
		ballPos = ball().pos;
	}

	Segment goal;
	goal.pt[0] = Point(Field_GoalWidth / 2, Field_Length);
	goal.pt[1] = Point(-Field_GoalWidth / 2, Field_Length);

	switch (gameState().state)
	{
	case GameState::Setup:
		if (mode == Mode_None)
		{
			chooseMode();
		}

		robot->move(Geometry2d::Point(0,Field_Length / 2 - 0.3), false); // stop at end enabled
		robot->face(ballPos);

		// Need this in case the kickoff is restarted (transition from Ready to Setup).
		// This should not normally happen but it helps with testing and sloppy referees.
		_kick.restart();
		break;

	case GameState::Ready:
	{
		// check for straight shot
		Geometry2d::Segment t;
		if (robot->kicker_available() && _kick.findShot(goal, t, false, 0.2))
		{
			_kick.setTargetGoal();
			_kick.use_chipper = false;
			_kick.run();
		} else
		{
			executeMode();
		}
		break;
	}

	default:
		break;
	}

	switch (mode) {
	case Mode_Kick:
		robot->addText("Mode: Kick");
		break;
	case Mode_KickLeftCorner:
		robot->addText("Mode: KickLeft");
		break;
	case Mode_KickRightCorner:
		robot->addText("Mode: KickRight");
		break;
	case Mode_Chip:
		robot->addText("Mode: Chip");
		break;
	case Mode_ChipLeftCorner:
		robot->addText("Mode: ChipLeft");
		break;
	case Mode_ChipRightCorner:
		robot->addText("Mode: ChipRight");
		break;
	case Mode_Bump:
		robot->addText("Mode: Bump");
		break;
	case Mode_None:
		robot->addText("Mode: None");
		break;
	}

	return gameState().state != GameState::Playing;
}
