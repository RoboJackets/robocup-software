/**
 *  This 'play' just tests drawing one the GUI
 */

#pragma once

#include <gameplay/Play.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class TestGUI: public Play
		{
			public:
				TestGUI(GameplayModule *gameplay);

				/** returns true if the play is currently applicable given gamestate */
				virtual bool applicable() { return true; }

				/** Called every frame */
				virtual bool run();

			protected:
				// Insert sub behaviors here as member variables
		};
	}
}
