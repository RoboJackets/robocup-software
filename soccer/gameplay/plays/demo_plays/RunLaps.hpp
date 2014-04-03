#pragma once

#include <gameplay/Play.hpp>
#include <Timeout.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class RunLaps: public Play
		{
			public:
				RunLaps(GameplayModule *gameplay);

				// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			protected:
				bool _targetingLeft;
				bool _arrived;
				Timeout _stabilizeTimeout;
		};
	}
}
