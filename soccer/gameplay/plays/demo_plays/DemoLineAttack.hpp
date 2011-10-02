#pragma once

#include "../../Play.hpp"

#include "../../behaviors/LineKick.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * Attack demo - choose one robot and target goal continuously
 * Uses the LineKick behavior
 */
class DemoLineAttack: public Play
{
public:
	static void createConfiguration(Configuration *cfg);

	DemoLineAttack(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::LineKick _kicker;

	static ConfigBool *_use_chipper;
	static ConfigInt *_kick_power;
};
} // \Gameplay
} // \Plays
