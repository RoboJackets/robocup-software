// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include "../../Behavior.hpp"
#include "../Kick.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Goalie: public Behavior
		{
			public:
				Goalie(GameplayModule *gameplay);
				~Goalie();

				virtual void run();
				virtual void start();

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
