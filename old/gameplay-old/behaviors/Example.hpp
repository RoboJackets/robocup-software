#pragma once

#include <gameplay/Behavior.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class ChangeMe: public Behavior
		{
			public:
				ChangeMe(GameplayModule *gameplay);
				
				virtual bool run();
		};
	}
}
