#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/behaviors/Bump.hpp>
#include <gameplay/PreventDoubleTouch.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class OurFreekick: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				OurFreekick(GameplayModule *gameplay);
				
				// this is a default free kick behavior - used for general restart kicks
				// returns 10 if applicable - specialized plays handle corner and goal cases
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				// Options for kicking the ball
				Behaviors::Kick _kicker;
				Behaviors::Bump _bump;

				Behaviors::Move _center1, _center2;
				Behaviors::Defender _defender1, _defender2;
				PreventDoubleTouch _pdt;

				static ConfigBool *_enableGoalLineShot;
				static ConfigBool *_enableLeftDownfieldShot;
				static ConfigBool *_enableRightDownfieldShot;
				static ConfigBool *_enableChipper;
				static ConfigDouble *_minChipRange;
				static ConfigDouble *_maxChipRange;
				static ConfigDouble *_avoidShot;
		};
	}
}
