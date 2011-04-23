#pragma once

#include <gameplay/Behavior.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class PivotKick: public SingleRobotBehavior
		{
			public:
				PivotKick(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				void restart();
				
				Geometry2d::Segment target;
				
			private:
				enum
				{
					State_Approach,
					State_Aim,
					State_Done
				} _state;
		};
	}
}
