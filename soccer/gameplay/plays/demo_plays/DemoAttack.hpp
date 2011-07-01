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
 * Uses parameters in the Config GUI to adjust flags
 */
class DemoAttack: public Play
{
public:
	DemoAttack(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Kick _kicker;

	// parameters to set in the GUI
	ConfigBool::shared_ptr _useChip;				// flag for chipping or kicking
	ConfigInt::shared_ptr _dribblerSpeed;   // sets the aim/kick dribbler speed
	ConfigInt::shared_ptr _kickPower;       // sets the speed to kick at
};
} // \Gameplay
} // \Plays
