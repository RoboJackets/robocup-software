#pragma once

#include "../Play.hpp"

#include <gameplay/PreventDoubleTouch.hpp>
#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/tactics/passing/PassingContext.hpp>
#include <gameplay/tactics/passing/DumbReceive.hpp>
#include <gameplay/tactics/passing/StablePass.hpp>

namespace Gameplay
{
	namespace Plays
	{

		class OurGoalKick_Pass: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				OurGoalKick_Pass(GameplayModule *gameplay);
				
				// same as free kick, but looks for direct and near our goal line
				// 1 for good, INF otherwise
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				StablePass _passer;
				DumbReceive _receiver1, _receiver2;
				Behaviors::Defender _defender1, _defender2;

				PassingContext _passCtxt;

				static ConfigDouble *_downFieldRange;
				// static ConfigDouble *_minChipRange;
				// static ConfigDouble *_maxChipRange;
				// static ConfigInt *_chipper_power;
				static ConfigInt *_kicker_power;

				static ConfigDouble *_field_length_mult;
				static ConfigDouble *_field_width_mult;
				static ConfigDouble *_field_length_off;
				static ConfigDouble *_field_width_off;
		};
	}
}
