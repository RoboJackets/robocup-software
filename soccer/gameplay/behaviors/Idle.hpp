#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class Idle: public Behavior
        {
            public:
                Idle(GameplayModule *gameplay, Role *role);
                
                virtual void run();
                
                virtual float score(Robot* r);
        };
    }
}
