#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/Mark.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TheirFreekick: public Play
		{
			public:
				TheirFreekick(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
			
			protected:
				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Mark _marking1, _marking2;
		};
	}
}
