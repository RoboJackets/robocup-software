#pragma once

#include "../Play.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class OurKickoff: public Play
		{
			public:
				OurKickoff(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool run();
		};
	}
}
