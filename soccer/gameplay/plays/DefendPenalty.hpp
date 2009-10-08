#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class DefendPenalty: public Play
		{
			public:
				DefendPenalty(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
