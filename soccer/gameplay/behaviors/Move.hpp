#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Move: public SingleRobotBehavior
		{
			public:
				Move(GameplayModule *gameplay);
				
				virtual bool run();
				
				Geometry2d::Point target;
				Geometry2d::Point face;
				float backoff;
		};
	}
}
