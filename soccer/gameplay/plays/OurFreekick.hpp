#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/PreventDoubleTouch.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class OurFreekick: public Play
		{
			public:
				OurFreekick(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				Behaviors::Kick _kicker;
				Behaviors::Move _center;
				Behaviors::Fullback _fullback1, _fullback2;
				PreventDoubleTouch _pdt;
		};
	}
}
