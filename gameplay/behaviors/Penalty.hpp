#pragma once

#include "../Behavior.hpp"
#include "Kick.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class Penalty: public Behavior
        {
            public:
                Penalty(GameplayModule *gameplay, Role *role);

                virtual void run();
                virtual bool done();
                virtual void start();
            private:
                Kick _kick;

        };
    }
}
