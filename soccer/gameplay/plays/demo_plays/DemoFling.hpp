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
	DemoFling(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Fling _fling;

	// parameters to set in the GUI
	ConfigInt::shared_ptr _dribblerSpeed;   // sets the aim/kick dribbler speed
	ConfigDouble::shared_ptr _spinAngularSpeed; // how fast to spin when flinging
};
} // \Gameplay
} // \Plays
