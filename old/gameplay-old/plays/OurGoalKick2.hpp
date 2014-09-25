#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/PreventDoubleTouch.hpp>

namespace Gameplay
{
	namespace Plays
	{

		class OurGoalKick2: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				OurGoalKick2(GameplayModule *gameplay);
				
				// same as free kick, but looks for direct and near our goal line
				// 1 for good, INF otherwise
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				Behaviors::Kick _kicker;
				Behaviors::Move _center1;
				Behaviors::Move _center2;
				Behaviors::Defender _defender1, _defender2;
				PreventDoubleTouch _pdt;

				bool hasChipper;

				static ConfigDouble *_downFieldRange;
				static ConfigDouble *_minChipRange;
				static ConfigDouble *_maxChipRange;
				static ConfigInt *_chipper_power;
				static ConfigInt *_kicker_power;

				static ConfigDouble *_field_length_mult;
				static ConfigDouble *_field_width_mult;
				static ConfigDouble *_field_length_off;
				static ConfigDouble *_field_width_off;
		};
	}
}
