#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TheirKickoff: public Play
		{
			public:
				TheirKickoff(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
			
			protected:
				Behaviors::Fullback _fullback1, _fullback2;
				Behaviors::Kick _back1, _back2;
		};
	}
}
