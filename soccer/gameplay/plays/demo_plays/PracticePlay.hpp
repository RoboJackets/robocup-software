/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class PracticePlay: public Play
		{
			public:
				PracticePlay(GameplayModule *gameplay);

				// Return INFNITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);


				/** Called every frame */
				virtual bool run();

			protected:
				Behaviors::Kick _kicker;
		};
	}
}
