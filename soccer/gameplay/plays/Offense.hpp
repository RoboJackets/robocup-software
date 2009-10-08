#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class Offense: public Play
		{
			public:
				Offense(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
