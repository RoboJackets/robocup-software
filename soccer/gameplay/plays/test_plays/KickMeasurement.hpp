#pragma once

#include <gameplay/Play.hpp>
#include <fstream>
#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Bump.hpp>
#include <gameplay/PreventDoubleTouch.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class KickMeasurement: public Play
		{
			public:
				typedef enum
				{
					State_Wait,
					State_Setup,
					State_Grab,
					State_Kick,
					State_Restart,
				} State;

				KickMeasurement(GameplayModule *gameplay);

				virtual bool run();

			protected:
				uint64_t _time;
				double _lastKickTime;
				double _fireDelta;
				bool _kicked;

				std::ofstream _file;

				Behaviors::Kick _kicker;
				uint8_t _kickPower;

				OurRobot *robot;

				Geometry2d::Point _robotPos;

			private:
				State _state;


		};
	}
}
