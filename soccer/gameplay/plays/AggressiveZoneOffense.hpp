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
				
				virtual bool applicable();
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
				
			protected:
				Behaviors::Fullback _fullback;
				Behaviors::ZoneOffense _offense; // uses three robots
		};
	}
}
