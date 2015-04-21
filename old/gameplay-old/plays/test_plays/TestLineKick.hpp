#pragma once

#include <gameplay/Play.hpp>
#include <fstream>
#include <gameplay/behaviors/LineKick.hpp>
#include <gameplay/behaviors/Bump.hpp>
#include <gameplay/PreventDoubleTouch.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TestLineKick: public Play
		{
			public:
				typedef enum
				{
					State_Wait,
					State_Ready,
					State_Kick,
				} State;

				TestLineKick(GameplayModule *gameplay);

				virtual bool run();

			protected:
				uint64_t _time;
				double _lastKickTime;
				double _fireDelta;
				bool _kicked;

				std::ofstream _file;

				//Behaviors::Kick _kicker;
				Behaviors::LineKick _LineKick;
				uint8_t _kickPower;

				OurRobot *robot;

				Geometry2d::Point _robotPos;

			private:
				State _state;


		};
	}
}
