#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		/**
		 *  Play for navigation challenge for the 2011 competition
		 *  Goal is to drive robots through a path of moving obstacles
		 *  to complete as many laps as possible.
		 *
		 *  From the description:
		 *  The aim of this technical challenge is examine the ability of robots to safely navigate in a dynamic environment.
		 *
		 *  As it is shown in the pictures, there are 6 robots acting as obstacles, two stationary and
		 *  four moving along a straight line. The approximate positions and trajectories are depicted
		 *  in the figures. The actual position may vary.
		 *
		 *  1.1) The maximum number of participating robots are limited to 3.
		 *
		 *  1.2) The participating robots must navigate between the two stationary obstacles as shown in pictures.
		 *
		 *  1.3) Whenever a robot touches an obstacle or its teammate it will receive a penalty (-1).
		 *
		 *  1.4) Each successful navigation cycle for each robot is scored (+1).
		 *
		 *  1.5) Robots who can perform the cycle carrying the ball score (+3).
		 *
		 *  1.6) The challenge time is 2 minutes.
		 *
		 *
		 */
		class NavChallenge2011: public Play
		{
			public:
				NavChallenge2011(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

		};
	}
}
