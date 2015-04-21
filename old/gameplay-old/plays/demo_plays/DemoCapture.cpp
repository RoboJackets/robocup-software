#include "DemoCapture.hpp"


using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoCapture, "Demos")

Gameplay::Plays::DemoCapture::DemoCapture(GameplayModule *gameplay):
Play(gameplay),
_capture(gameplay)
{
	// Capture defaults to aiming at goal center
}

bool Gameplay::Plays::DemoCapture::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_capture.robot, available, Geometry2d::Point());

	// Geometry2d::Point ballPos = ball().pos;

	// if we have captured the ball, reset to keep trying to capture
	if (_capture.done() &&  ball().valid)
	{
		_capture.restart();
	}

	_capture.run();

	return true;
}
