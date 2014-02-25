#pragma once

#include <gameplay/Play.hpp>
#include <Pid.hpp>

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

				static void createConfiguration(Configuration *cfg);

				/**
				 * @param 1 The time (in seconds) since starting the path
				 * @param 2 target position out
				 * @param 3 target velocity out
				 * @return whether or not the path exists at the given time
				 */
				std::function<bool(float, Geometry2d::Point &, Geometry2d::Point &)> path;

			protected:
				bool testStarted;
				OurRobot *robot;

				/** Whether we're going from ptA to ptB or ptB to ptA */
				bool reverseLap;
				uint64_t lapStartTime;

				Pid _pidControllerX;
				Pid _pidControllerY;

				static ConfigDouble *_pid_p;
				static ConfigDouble *_pid_i;
				static ConfigDouble *_pid_d;
				static ConfigDouble *_v_p;
		};
	}
}
