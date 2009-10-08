#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class TheirFreekick: public Play
		{
			public:
				TheirFreekick(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
