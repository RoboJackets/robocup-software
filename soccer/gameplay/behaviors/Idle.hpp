#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class Idle: public Behavior
        {
            public:
                Idle(GameplayModule *gameplay);
                
                virtual bool run();
		virtual bool assign(std::set<Robot *> &available);
        };
    }
}
