#pragma once

#include "../../Play.hpp"

#include "../../behaviors/YankChip.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * This play just drives the robot to set up and attempt yank-chip shots
 *
 * Uses parameters in the Config GUI to adjust flags
 */
class DemoYankChip: public Play
{
public:
	static void createConfiguration(Configuration *cfg);

	DemoYankChip(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::YankChip _yank;

	// parameters to set in the GUI
	static ConfigInt *_dribblerSpeed;   // sets the aim/kick dribbler speed
	static ConfigDouble *_backupDist;   // how to far backup before driving forward to chip
	static ConfigBool * _useTarget;      // flag to attempting aiming
};
} // \Gameplay
} // \Plays
