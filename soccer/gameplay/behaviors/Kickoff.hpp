#pragma once

#include "../Behavior.hpp"

#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class Kickoff: public SingleRobotBehavior
		{
			public:
				Kickoff(GameplayModule *gameplay);
				
				virtual bool run();

				Behaviors::Kick kick;
		};
	}
}
