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

	/**
	 * Takes the first robot as a kicker, ignores the rest
	 */
	virtual bool assign(std::set<Robot *> &available);

	/** default run */
	virtual bool run();

protected:
	Behaviors::OneTouchKick _kicker;
};
} // \Gameplay
} // \Plays
