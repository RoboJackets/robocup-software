#pragma once

#include <gameplay/Play.hpp>
#include <gameplay/ActionBehavior.hpp>
#include <gameplay/tactics/passing/PassReceiveTactic.hpp>

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
