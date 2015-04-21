#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Mark.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TheirFreekick: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);
				TheirFreekick(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				bool _assignedMark1;
				bool _assignedMark2;
				bool _assignedMark3;

				Behaviors::Defender _defender1, _defender2;
				Behaviors::Mark _marking1, _marking2, _marking3;
		};
	}
}
