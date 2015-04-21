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
		/**
		 * Sends a corner kick across the opponent's goal with
		 * a
		 */
		class OurCornerKick: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				OurCornerKick(GameplayModule *gameplay);
				
				// same as free kick, but looks for direct and near their goal line
				// 1 for good, INF otherwise
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				// always takes a chipper
				Behaviors::Kick _kicker;
				Behaviors::Move _center1;
				Behaviors::Move _center2;
				Behaviors::Defender _defender1, _defender2;
				PreventDoubleTouch _pdt;

				static ConfigDouble *_targetSegmentWidth;
				static ConfigDouble *_minChipRange;
				static ConfigDouble *_maxChipRange;
				static ConfigInt *_chipper_power;
		};
	}
}
