#pragma once

#include <gameplay/Behavior.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class LineKick: public SingleRobotBehavior
		{
			public:
				LineKick(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				void restart();
				
				Geometry2d::Point target;
				
			private:
				enum
				{
					State_Setup,
					State_Charge,
					State_Done
				} _state;
		};
	}
}
