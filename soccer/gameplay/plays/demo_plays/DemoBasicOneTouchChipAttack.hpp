#pragma once

#include "../../Play.hpp"

#include "../../behaviors/OneTouchKick.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * This play just drives all robots to kick the ball continuously
 * Useful for testing basic operation of the system, and should
 * probably only be used with 1-2 robots on the field, as it
 * will get chaotic
 *
 * This variant uses chip kicking and one touch kicking
 */
class DemoBasicOneTouchChipAttack: public Play
{
public:
	DemoBasicOneTouchChipAttack(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::OneTouchKick _kicker;
};
} // \Gameplay
} // \Plays
