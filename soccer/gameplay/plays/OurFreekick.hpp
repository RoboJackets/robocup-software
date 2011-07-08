#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/positions/Fullback.hpp>
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
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				// Options for kicking the ball
				Behaviors::Kick _kicker;
				Behaviors::Bump _bump;

				Behaviors::Move _center;
				Behaviors::Fullback _fullback1, _fullback2;
				PreventDoubleTouch _pdt;

				static ConfigBool *_enableGoalLineShot;
				static ConfigBool *_enableLeftDownfieldShot;
				static ConfigBool *_enableRightDownfieldShot;
				static ConfigBool *_enableChipper;
				static ConfigDouble *_minChipRange;
				static ConfigDouble *_maxChipRange;
		};
	}
}
