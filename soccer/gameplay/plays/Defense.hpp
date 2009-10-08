#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class Defense: public Play
		{
			public:
				Defense(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
