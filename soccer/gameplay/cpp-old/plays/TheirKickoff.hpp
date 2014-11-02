#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Idle.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TheirKickoff: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);
				TheirKickoff(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				Behaviors::Defender _defender1, _defender2;
				Behaviors::Idle _idle;
		};
	}
}
