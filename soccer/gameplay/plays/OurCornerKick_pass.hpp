#pragma once

#include "../Play.hpp"
#include <gameplay/tactics/DumbReceive.hpp>
#include <gameplay/tactics/StablePass.hpp>
#include <gameplay/behaviors/PivotKick.hpp>
#include <gameplay/behaviors/positions/Fullback.hpp>
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
		class OurCornerKick_Pass: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				OurCornerKick_Pass(GameplayModule *gameplay);
				
				// same as free kick, but looks for direct and near their goal line
				// 1 for good, INF otherwise
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				Behaviors::Fullback _fullback1, _fullback2;


				bool _passDone;

				PreventDoubleTouch _pdt;

				StablePass _passer;
				DumbReceive _receiver1;
				DumbReceive _receiver2;

				static ConfigDouble *_targetSegmentWidth;
		};
	}
}
