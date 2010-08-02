#include "WheelController.hpp"

namespace Motion {

WheelController::WheelController(SystemState *state, const ConfigFile::MotionModule& cfg)
: _state(state), _config(cfg)
{
}

void WheelController::run()
{
	for (size_t i=0; i < Constants::Robots_Per_Team; ++i)
	{
		// get the velocity command from the motion robot
		// convert to wheel speeds
		// assign to radio packet
	}
}

}
