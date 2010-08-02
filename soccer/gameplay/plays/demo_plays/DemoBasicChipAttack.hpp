#pragma once

#include "../../Play.hpp"

#include "../../behaviors/Kick.hpp"

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
 * This variant uses chip kicking and only works for 2010 robots
 */
class DemoBasicChipAttack: public Play
{
public:
	DemoBasicChipAttack(GameplayModule *gameplay);

	/**
	 * Takes the first robot as a kicker, ignores the rest
	 */
	virtual bool assign(std::set<Robot *> &available);

	/** default run */
	virtual bool run();

protected:
	Behaviors::Kick _kicker;
};
} // \Gameplay
} // \Plays
