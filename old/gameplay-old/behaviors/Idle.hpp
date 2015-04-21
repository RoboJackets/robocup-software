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
            	~Idle();
                
                virtual bool run();

                // Lines up the robots
                virtual bool run(const Geometry2d::Segment& line);
				
				std::set<OurRobot *> robots;
        };
    }
}
