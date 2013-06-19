#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/tactics/ActionBehavior.hpp>
#include <gameplay/tactics/PassReceiveTactic.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class CatPassDemo: public Play
		{
			public:
				CatPassDemo(GameplayModule *gameplay);
				~CatPassDemo();

				virtual bool run();

			protected:
				bool initPositions();

				std::vector<PassReceiveTactic> passArray;

				int currentPass;
		};
	}
}
