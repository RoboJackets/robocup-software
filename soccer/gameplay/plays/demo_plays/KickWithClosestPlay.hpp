/**
 *	Example play: This play is causes the closest robot to the ball
 *  to approach and kick it. Its logic isn't perfect. It generally
 *  works when the robot approaches the ball from the left.
 */

#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class KickWithClosestPlay: public Play
		{
			public:
				KickWithClosestPlay(GameplayModule *gameplay);

				// Return INFNITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);


				/** Called every frame */
				virtual bool run();
				bool reached;



		};
	}
}
