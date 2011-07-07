#pragma once

#include "../../Play.hpp"

#include "../../behaviors/Bump.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * This play just drives the robot to take Bump shots at the goal,
 *
 * Uses parameters in the Config GUI to adjust flags
 */
class DemoBump: public Play
{
public:
	static void createConfiguration(Configuration *cfg);

	DemoBump(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Bump _bump;

};
} // \Gameplay
} // \Plays
