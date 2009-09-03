#pragma once

#include "../../Behavior.hpp"
#include "../Dribble.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Dribble;

		class KickerCalibration: public Behavior
		{
			public:
				KickerCalibration(GameplayModule *gameplay, Role *role);

				virtual void start();
				virtual void run();
				virtual bool done();

			protected:
				enum State
				{
					Wait,
					Setup,
					Shoot,
					Record,
					Done
				};

				FILE *fp;
				State _state;
				Dribble _dribble;
				int _strength;
				uint64_t _oldTime;
				Geometry2d::Point _startPos;
				Geometry2d::Point _oldBallPos;
		};
	}
}
