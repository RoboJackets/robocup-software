#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		/** This behavior will chip the ball at a target */
		class Chip: public Behavior
		{
			public:
				Chip(GameplayModule *gameplay);
				
				virtual bool run();
				
				virtual bool assign(std::set<Robot *> &available);

				virtual float score(Robot* robot);
		};
	}
}
