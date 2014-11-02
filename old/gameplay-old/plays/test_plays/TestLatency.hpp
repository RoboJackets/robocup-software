#pragma once

#include <gameplay/Play.hpp>
#include <fstream>

namespace Gameplay
{
	namespace Plays
	{
		class TestLatency: public Play
		{
			public:
				TestLatency(GameplayModule *gameplay);

				virtual bool run();

			protected:
				uint64_t _startTime;
				bool _first;
				std::ofstream _file;
		};
	}
}
