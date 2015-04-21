#pragma once

#include "../../Play.hpp"

#include "../../behaviors/Fling.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * This play just drives the robot to take fling shots at the goal,
 * with options for fixing the approach direction or letting the
 * behavior decide automatically
 *
 * Uses parameters in the Config GUI to adjust flags
 */
class DemoFling: public Play
{
public:
	static void createConfiguration(Configuration *cfg);

	DemoFling(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Fling _fling;

	// parameters to set in the GUI
	static ConfigInt *_dribblerSpeed;   // sets the aim/kick dribbler speed
	static ConfigDouble *_spinAngularSpeed; // how fast to spin when flinging
};
} // \Gameplay
} // \Plays
