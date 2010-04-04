#pragma once

#include "../Behavior.hpp"
#include "Kick.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Penalty: public Behavior
		{
			public:
				Penalty(GameplayModule *gameplay);

				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
				
			private:
				Kick _kick;
		};
	}
}
