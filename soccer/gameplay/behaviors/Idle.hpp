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
                
				virtual void assign(std::set<Robot *> &available);
                virtual bool run();
        };
    }
}
