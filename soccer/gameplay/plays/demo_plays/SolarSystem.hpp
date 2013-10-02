#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class SolarSystem: public Play
		{
			public:
				SolarSystem(GameplayModule *gameplay);

				static float score(GameplayModule *gameplay);
				virtual bool run();
		};
	}
}
