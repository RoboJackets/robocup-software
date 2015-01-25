#include "DemoBump.hpp"


using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoBump, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoBump)
	}
}

void Gameplay::Plays::DemoBump::createConfiguration(Configuration* cfg)
{
}

Gameplay::Plays::DemoBump::DemoBump(GameplayModule *gameplay):
Play(gameplay),
_bump(gameplay)
{
}

bool Gameplay::Plays::DemoBump::run()
{
	Geometry2d::Point ballPos = ball().pos;

	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_bump.robot, available, ballPos);

	// if we have kicked, we want to reset
	if (_bump.done() &&  ball().valid &&
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_bump.restart();
	}

	_bump.run();

	return true;
}
