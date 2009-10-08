#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class Kickoff: public Behavior
        {
            public:
                Kickoff(GameplayModule *gameplay);
                
                virtual bool run();
        };
    }
}
