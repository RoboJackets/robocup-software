#pragma once

#include "../../Play.hpp"

#include "../../behaviors/Yank.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * This play just drives the robot to set up and attempt yank shots
 * at the goal. The behavior should choose turn direction correctly to
 * move itself out of the way.
 *
 * Uses parameters in the Config GUI to adjust flags
 */
class DemoYank: public Play
{
public:

	static void createConfiguration(Configuration *cfg);

	DemoYank(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Yank _yank;

	// parameters to set in the GUI
	static ConfigInt *_dribblerSpeed;   // sets the aim/kick dribbler speed
	static ConfigBool * _enableBump;     // uses a bump before yanking
};
} // \Gameplay
} // \Plays
