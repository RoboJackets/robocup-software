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
				
				std::set<OurRobot *> robots;
        };
    }
}
