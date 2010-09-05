// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include "../../Behavior.hpp"
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	class WindowEvaluator;
	
	namespace Behaviors
	{
		class Goalie: public SingleRobotBehavior
		{
			public:
				Goalie(GameplayModule *gameplay);
				~Goalie();

				// Takes one robot out of <available> and makes it the goalie.
				// This will be called every frame.
				void assign(std::set<OurRobot *> &available);
				
				virtual bool run();

			protected:
				typedef enum
				{
					Defend,
					Clear
				} State;
				
				WindowEvaluator* _win;
				
				Kick _kick;
				
				State _state;
				
				unsigned int _index;
		};
	}
}
