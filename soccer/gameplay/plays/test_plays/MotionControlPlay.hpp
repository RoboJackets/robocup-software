#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * This is a test play to try out some motion control ideas.  It currently just runs a
		 * trapezoidal motion/velocity profile on a single robot on our side of the field, making
		 * it more from one side to the other.
		 */
		class MotionControlPlay: public Play
		{
			public:
				MotionControlPlay(GameplayModule *gameplay);

				// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			protected:
				bool testStarted;
				OurRobot *robot;

				/** Whether we're going from ptA to ptB or ptB to ptA */
				bool reverseLap;
				uint64_t lapStartTime;
				float lastTime;
				float lastVelocityCommand;
				float lastPosition;
		};
	}
}
