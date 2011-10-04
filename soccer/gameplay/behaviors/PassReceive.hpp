#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class PassReceive: public TwoRobotBehavior
		{
			public:
				static void createConfiguration(Configuration *cfg);
				PassReceive(GameplayModule *gameplay);
				
				virtual bool run();

		};
	}
}
