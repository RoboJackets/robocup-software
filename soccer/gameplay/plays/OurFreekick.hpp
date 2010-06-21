#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/positions/Forward.hpp>
#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/Move.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class OurFreekick: public Play
		{
			public:
				OurFreekick(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
			
			protected:
				Behaviors::Kick _kicker;
				Behaviors::Move _center;
				Behaviors::Fullback _fullback1, _fullback2;
		};
	}
}
