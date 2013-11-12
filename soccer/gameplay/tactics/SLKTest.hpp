#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/ActionBehavior.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class SLKTest: public Play
		{
			public:
				SLKTest(GameplayModule *gameplay);
				~SLKTest();

				virtual bool run();

			protected:
				ActionBehavior *_passer;
				ActionBehavior *_receiver;
		};
	}
}
