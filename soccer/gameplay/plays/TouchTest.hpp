#pragma once

#include "../Play.hpp"

#include "../behaviors/TouchKick.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * Attack demo - choose one robot and target goal continuously
 * Uses the LineKick behavior
 */
class TouchTest: public Play
{
public:
	static void createConfiguration(Configuration *cfg);

	TouchTest(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::TouchKick _kicker;

	static ConfigBool *_use_chipper;
	static ConfigInt *_kick_power;
};
} // \Gameplay
} // \Plays
