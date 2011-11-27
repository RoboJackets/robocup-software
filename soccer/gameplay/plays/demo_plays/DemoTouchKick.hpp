#pragma once

#include "../../Play.hpp"

#include "../../behaviors/TouchKick.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * Tests the TouchKick behavior
 */
class DemoTouchKick: public Play
{
public:
	static void createConfiguration(Configuration *cfg);

	DemoTouchKick(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::TouchKick _kicker;

	static ConfigBool *_use_chipper;
	static ConfigInt *_kick_power;
};
} // \Gameplay
} // \Plays
