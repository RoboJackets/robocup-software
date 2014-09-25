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
		 * Sends a goal kick towards the goal if its open with kicker,
		 * otherwise, chips ball to opposite side of field
		 * and sends another robot to go intercept
		 */
		class OurGoalKick: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				OurGoalKick(GameplayModule *gameplay);
				
				// same as free kick, but looks for direct and near our goal line
				// 1 for good, INF otherwise
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				// always takes a chipper - will shoot if open shots
				Behaviors::Kick _kicker;
				Behaviors::Move _center1;
				Behaviors::Move _center2;
				Behaviors::Defender _defender1, _defender2;
				PreventDoubleTouch _pdt;

				static ConfigDouble *_downFieldRange;
				static ConfigDouble *_minChipRange;
				static ConfigDouble *_maxChipRange;
				static ConfigInt *_chipper_power;
		};
	}
}
