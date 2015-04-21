#pragma once

#include "../../Play.hpp"

#include "../../behaviors/TouchKick.hpp"
#include "../../behaviors/Kick.hpp"

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
	Behaviors::Kick _passer;

	Geometry2d::Point _passTarget;

	static ConfigBool *_use_chipper;
	static ConfigInt *_kick_power;
};
} // \Gameplay
} // \Plays
