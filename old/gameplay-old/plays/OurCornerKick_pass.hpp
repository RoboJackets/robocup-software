#pragma once

#include "../Play.hpp"
#include <gameplay/tactics/passing/DumbReceive.hpp>
#include <gameplay/tactics/passing/StablePass.hpp>
#include <gameplay/behaviors/PivotKick.hpp>
#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/PreventDoubleTouch.hpp>
#include <gameplay/tactics/passing/PassingContext.hpp>

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
				Behaviors::Defender _defender1, _defender2;






				uint64_t _startTime;
				uint64_t _choosinessTimeout;	//	after this many microseconds?, we'll stop choosing between receive points and make it happen
				bool _firstRun;

				PreventDoubleTouch _pdt;

				PassingContext _passCtxt;
				StablePass _passer;
				DumbReceive _receiver1;
				DumbReceive _receiver2;

				static ConfigDouble *_targetSegmentWidth;

				bool _passingToFirstReceiver;
				static ConfigDouble *_receiverChoiceHysterisis;
		};
	}
}
