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
 */
class DemoBasicAttack: public Play
{
public:
	DemoBasicAttack(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Kick _kicker;
};
} // \Gameplay
} // \Plays
