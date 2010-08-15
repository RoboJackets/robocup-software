#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/ZoneOffense.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class AggressiveZoneOffense: public Play
		{
			public:
				AggressiveZoneOffense(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				Behaviors::Fullback _fullback;
				Behaviors::ZoneOffense _offense; // uses three robots
		};
	}
}
