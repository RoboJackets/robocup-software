#pragma once

#include "../../Play.hpp"

#include "../../behaviors/Capture.hpp"

namespace Gameplay
{
namespace Plays
{
/**
 * Demos continuous capturing with a single robot
 * Uses the Capture behavior
 */
class DemoCapture: public Play
{
public:
	DemoCapture(GameplayModule *gameplay);

	virtual bool run();

protected:
	Behaviors::Capture _capture;
};
} // \Gameplay
} // \Plays
