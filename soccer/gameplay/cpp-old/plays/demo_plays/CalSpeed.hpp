#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class CalSpeed: public Play
		{
			public:
				CalSpeed(GameplayModule *gameplay);

				// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			protected:
				int _speed;
				uint64_t _startTime;
				uint64_t _lastTime;
				float _lastAngle;
				float _average;
				int _count;
		};
	}
}
