#pragma once

#include "../../Play.hpp"

#include "../../behaviors/LineKick.hpp"

namespace Gameplay
{
	namespace Plays
	{
		class DemoKickMeasurement: public Play
		{
			public:
			static void createConfiguration(Configuration *cfg);

				DemoKickMeasurement(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			//
			protected:
				Geometry2d::Point position [100];
				Geometry2d::Point velocity [100];
				int time [100];
				bool kicked;

				int msr_index;

			//DemoLineAttack
			protected:
				Behaviors::LineKick _kicker;

				static ConfigBool *_use_chipper;
				static ConfigInt *_kick_power;
		};
	}
}
