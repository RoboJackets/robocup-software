#pragma once

#include <gameplay/Play.hpp>
#include <Pid.hpp>
#include <vector>

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * This is a test play to try out angle control.  It sets a face target for the bot.  Waits a couple seconds,
		 * then sets a new face target 180deg away and repeats.  This helps us tune the pid controller.
		 */
		class AngleControlTest: public Play
		{
			public:
				AngleControlTest(GameplayModule *gameplay);

				// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();


			protected:
				uint64_t _targetStartTime;	//	the time we started this face target
				int _targetIndex;
				std::vector<Geometry2d::Point> _targets;

				OurRobot *_robot;
		};
	}
}
