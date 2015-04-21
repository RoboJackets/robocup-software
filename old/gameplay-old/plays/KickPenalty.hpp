#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Move.hpp>
#include <gameplay/behaviors/Penalty.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class KickPenalty: public Play
		{
			public:
				KickPenalty(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				
				virtual bool run();
			
			protected:
				Behaviors::Penalty _kicker;
				Behaviors::Move _idle1, _idle2, _idle3, _idle4;
		};
	}
}
