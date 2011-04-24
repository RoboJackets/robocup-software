#pragma once

#include "../../Play.hpp"

#include "../../behaviors/PivotKick.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * Demos continuous attack with a single robot
 * Uses the PivotKick behavior
 */
class DemoBasicPivotAttack: public Play
{
public:
	DemoBasicPivotAttack(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::PivotKick _kicker;
};
} // \Gameplay
} // \Plays
