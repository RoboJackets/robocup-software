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
class DemoBasicLineAttack: public Play
{
public:
	DemoBasicLineAttack(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::LineKick _kicker;
};
} // \Gameplay
} // \Plays
