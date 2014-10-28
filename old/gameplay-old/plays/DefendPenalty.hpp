#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Move.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class DefendPenalty: public Play
		{
			public:
				DefendPenalty(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				
				virtual bool run();
			
			protected:
				Behaviors::Move _idle1, _idle2, _idle3, _idle4, _idle5;
		};
	}
}
