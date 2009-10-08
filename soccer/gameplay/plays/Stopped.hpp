#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Idle;
	}
	
	namespace Plays
	{
		class Stopped: public Play
		{
			public:
				Stopped(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
