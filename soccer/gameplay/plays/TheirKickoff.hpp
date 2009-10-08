#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class TheirKickoff: public Play
		{
			public:
				TheirKickoff(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
