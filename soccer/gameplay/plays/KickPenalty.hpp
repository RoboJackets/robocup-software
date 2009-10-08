#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class KickPenalty: public Play
		{
			public:
				KickPenalty(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
