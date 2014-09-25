#pragma once

#include "../Behavior.hpp"
#include "Kick.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Penalty: public SingleRobotBehavior
		{
			public:
				Penalty(GameplayModule *gameplay);

				virtual bool run();
				
			private:
				Kick _kick;
		};
	}
}
