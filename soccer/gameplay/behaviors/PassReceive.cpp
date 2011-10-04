#include "PassReceive.hpp"

namespace Gameplay {
namespace Behaviors {
REGISTER_CONFIGURABLE(PassReceive)
}
}

void Gameplay::Behaviors::PassReceive::createConfiguration(Configuration *cfg)
{

}

Gameplay::Behaviors::PassReceive::PassReceive(GameplayModule *gameplay):
	TwoRobotBehavior(gameplay)
{
}

bool Gameplay::Behaviors::PassReceive::run()
{
	return true;
}
