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
					State_Ready,
					State_Grab,
					State_Kick,
					State_Measure,
					State_Restart,
				} State;

				static void createConfiguration(Configuration *cfg);

				KickMeasurement(GameplayModule *gameplay);

				virtual bool run();

			protected:
				uint64_t _time;
				double _lastKickTime;
				double _fireDelta;
				double _lastGrabTime;
				bool _kicked;
				bool _grabbed;

				std::ofstream _file;

				Behaviors::Kick _kicker;
				uint8_t _kickPower;

				OurRobot *robot;

				Geometry2d::Point _robotPos;

			private:
				State _state;

				static ConfigBool   *_ready_signal;
		};
	}
}
