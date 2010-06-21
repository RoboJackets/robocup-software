#pragma once

#include "../Play.hpp"
#include <gameplay/behaviors/Move.hpp>
#include <gameplay/behaviors/Kickoff.hpp>

namespace Gameplay
{
	namespace Plays
	{
		class OurKickoff: public Play
		{
			public:
				OurKickoff(GameplayModule *gameplay);
				
				virtual bool applicable();
				virtual bool assign(std::set<Robot *> &available);
				virtual bool run();
			
			protected:
				bool _preventDoubleTouch;
				Behaviors::Kickoff _kicker;
				Behaviors::Move _idle1, _idle2, _idle3;
		};
	}
}
