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
	static void createConfiguration(Configuration *cfg);

	DemoAttack(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Kick _kicker;

	// parameters to set in the GUI
	static ConfigBool * _useChip;				// flag for chipping or kicking
	static ConfigBool * _useLineKick;   // switches to line kick mode
	static ConfigBool * _calculateChipPower;				// use varying chip power
	static ConfigInt *_dribblerSpeed;   // sets the aim/kick dribbler speed
	static ConfigInt *_kickPower;       // sets the speed to kick at
	static ConfigDouble *_minChipRange; // minimum range at which to chip
	static ConfigDouble *_maxChipRange; // maximum range at which to chip
};
} // \Gameplay
} // \Plays
