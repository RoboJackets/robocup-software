#include "DemoFling.hpp"


using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoFling, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoFling)
	}
}

ConfigInt *Gameplay::Plays::DemoFling::_dribblerSpeed;
ConfigDouble	 *Gameplay::Plays::DemoFling::_spinAngularSpeed;

void Gameplay::Plays::DemoFling::createConfiguration(Configuration* cfg)
{
	_dribblerSpeed = new ConfigInt(cfg, "DemoFling/Dribber Speed", 127);
	_spinAngularSpeed  = new ConfigDouble(cfg, "Fling/Spin Angular Speed", 2.0);
}

Gameplay::Plays::DemoFling::DemoFling(GameplayModule *gameplay):
Play(gameplay),
_fling(gameplay)
{
}

bool Gameplay::Plays::DemoFling::run()
{
	Geometry2d::Point ballPos = ball().pos;

	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_fling.robot, available, ballPos);

	// if we have kicked, we want to reset
	if (_fling.done() &&  ball().valid &&
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_fling.restart();
	}

	// set flags from parameters
	_fling.dribble_speed = *_dribblerSpeed;
	_fling.spin_speed = *_spinAngularSpeed;

	_fling.run();

	return true;
}
