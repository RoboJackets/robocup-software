/**
 *  Example play: This is a template for a play.
 *  To use, implement the functions and add the necessary member variables
 *  and do a test replacement for ExamplePlay with whatever name you want.
 */

#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class ExamplePlay: public Play
		{
			public:
				ExamplePlay(GameplayModule *gameplay);

				// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
				static float score(GameplayModule *gameplay);

				/** Called every frame */
				virtual bool run();

			protected:
				// Insert sub behaviors here as member variables
		};
	}
}
