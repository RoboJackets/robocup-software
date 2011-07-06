#include "Kickoff.hpp"
#include <algorithm>

using namespace std;

namespace Gameplay {
namespace Behaviors {
REGISTER_CONFIGURABLE(Kickoff)
}
}

ConfigBool *Gameplay::Behaviors::Kickoff::_enableRandomKick;
ConfigBool *Gameplay::Behaviors::Kickoff::_enableFling;
ConfigBool *Gameplay::Behaviors::Kickoff::_enableChip;
ConfigBool *Gameplay::Behaviors::Kickoff::_enableBumpYank;
ConfigDouble *Gameplay::Behaviors::Kickoff::_chipMinRange;
ConfigDouble *Gameplay::Behaviors::Kickoff::_chipMaxRange;

void Gameplay::Behaviors::Kickoff::createConfiguration(Configuration *cfg)
{
	_enableRandomKick = new ConfigBool(cfg, "Kickoff/Enable Random Kick Mode", false);
	_enableFling = new ConfigBool(cfg, "Kickoff/Enable Flinging", false);
	_enableChip = new ConfigBool(cfg, "Kickoff/Enable Chipping", false);
	_enableBumpYank = new ConfigBool(cfg, "Kickoff/Enable BumpYank", false);
	_chipMinRange = new ConfigDouble(cfg, "Kickoff/Chip Min Range", 0.3);
	_chipMaxRange = new ConfigDouble(cfg, "Kickoff/Chip Max Range", 2.0);
}

Gameplay::Behaviors::Kickoff::Kickoff(GameplayModule *gameplay):
SingleRobotBehavior(gameplay),
_kick(gameplay), _fling(gameplay), _yank(gameplay)
{
	mode = Mode_Kick;
	_mode_chosen = false;

	// seed random number generator
	srand(state()->timestamp);
}

void Gameplay::Behaviors::Kickoff::chooseMode() {
	if (*_enableRandomKick)
	{
		// add enabled modes to pool and pick one at random
		vector<KickoffMode> modes;
		modes.push_back(Mode_Kick);
		if (*_enableBumpYank)
		{
			modes.push_back(Mode_BumpYankLeft);
			modes.push_back(Mode_BumpYankRight);
		}

		if (*_enableChip)
		{
			modes.push_back(Mode_Chip);
		}

		if (*_enableFling)
		{
			modes.push_back(Mode_FlingLeft);
			modes.push_back(Mode_FlingRight);
		}

		random_shuffle(modes.begin(), modes.end());
		mode = modes.front();

	} else
	{
		// if not random, pick first on list that is available
		mode = Mode_Kick;
		if (*_enableBumpYank)
		{
			mode = Mode_BumpYankLeft;
		}

		if (*_enableFling)
		{
			mode = Mode_FlingRight;
		}

		if (*_enableChip)
		{
			mode = Mode_Chip;
		}
	}
}

void Gameplay::Behaviors::Kickoff::executeMode() {
	switch (mode) {
	case Mode_Kick:
		_kick.use_chipper = false;
		_kick.run();
		break;
	case Mode_Chip:
		_kick.use_chipper = true;
		_kick.maxChipRange = *_chipMaxRange;
		_kick.minChipRange = *_chipMinRange;
		_kick.run();
		break;
	case Mode_FlingLeft:
		_fling.target = Geometry2d::Point(Field_Width / 2, Field_Length * .75);
		_fling.run();
		break;
	case Mode_FlingRight:
		_fling.target = Geometry2d::Point(-Field_Width / 2, Field_Length * .75);
		_fling.run();
		break;
	case Mode_BumpYankLeft:
		_yank.target = Geometry2d::Segment(
				Geometry2d::Point(Field_Width / 2, Field_Length * .25),
				Geometry2d::Point(            0.0, Field_Length * .25));
		_yank.enable_bump = true;
		_yank.run();
		break;
	case Mode_BumpYankRight:
		_yank.target = Geometry2d::Segment(
				Geometry2d::Point(-Field_Width / 2, Field_Length * .25),
				Geometry2d::Point(             0.0, Field_Length * .25));
		_yank.enable_bump = true;
		_yank.run();
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
	_fling.robot = robot;
	_yank.robot = robot;

	// Use the real ball position if we have it.  Otherwise, assume the middle of the field.
	Geometry2d::Point ballPos(0, Field_Length / 2);
	if (ball().valid)
	{
		ballPos = ball().pos;
	}

	switch (gameState().state)
	{
	case GameState::Setup:
		if (!_mode_chosen)
		{
			chooseMode();
			_mode_chosen = true;
		}

		robot->move(Geometry2d::Point(0,Field_Length / 2 - 0.3), false); // stop at end enabled
		robot->face(ballPos);

		// Need this in case the kickoff is restarted (transition from Ready to Setup).
		// This should not normally happen but it helps with testing and sloppy referees.
		_kick.restart();
		break;

	case GameState::Ready:
		executeMode();
		break;

	default:
		break;
	}

	switch (mode) {
	case Mode_Kick:
		robot->addText("Mode: Kick");
		break;
	case Mode_Chip:
		robot->addText("Mode: Chip");
		break;
	case Mode_FlingLeft:
		robot->addText("Mode: FlingLeft");
		break;
	case Mode_FlingRight:
		robot->addText("Mode: FlingRight");
		break;
	case Mode_BumpYankLeft:
		robot->addText("Mode: BumpYankLeft");
		break;
	case Mode_BumpYankRight:
		robot->addText("Mode: BumpYankRight");
		break;
	}

	return gameState().state != GameState::Playing;
}
