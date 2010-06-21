#pragma once

#include "../Behavior.hpp"

#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
    namespace Behaviors
    {
        class Kickoff: public Behavior
        {
            public:
                Kickoff(GameplayModule *gameplay);
                
                virtual bool run();
		virtual bool assign(std::set<Robot *> &available);
	
		Behaviors::Kick kick;
        };
    }
}
