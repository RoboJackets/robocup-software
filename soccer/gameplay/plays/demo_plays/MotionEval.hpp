#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class MotionEval: public Play
		{
			public:
				MotionEval(GameplayModule *gameplay);

				// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			protected:
				std::vector<Geometry2d::Point> _points;
				
				// Which point we're driving to
				int _target;
				
				bool _reached;
				uint64_t _reachedTime;
				float _lastAngle;
				uint64_t _lastTime;
		};
	}
}
