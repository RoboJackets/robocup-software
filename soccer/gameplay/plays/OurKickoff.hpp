#pragma once

#include "../Play.hpp"
#include <gameplay/PreventDoubleTouch.hpp>
#include <gameplay/behaviors/Idle.hpp>
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/behaviors/Kickoff.hpp>

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * General Kickoff Play - just moves robots into position,
		 * actual kicking handled by the Kickoff Behavior
		 */
		class OurKickoff: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);
				OurKickoff(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
			
			protected:
				Behaviors::Kickoff _kicker;
				Behaviors::Move _idle1, _idle2, _idle3, _idle4;
				PreventDoubleTouch _pdt;

				static ConfigInt *_kick_power;
				static ConfigInt *_chip_power;
		};
	}
}
