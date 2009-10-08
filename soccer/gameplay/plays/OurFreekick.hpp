#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class OurFreekick: public Play
		{
			public:
				OurFreekick(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
