#pragma once

#include "../../Behavior.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class MotionLeader: public SingleRobotBehavior
        {
            public:
                MotionLeader(GameplayModule *gameplay);

                virtual bool run();

            protected:
                std::shared_ptr<Obstacle> centerObstacle[2];
                std::shared_ptr<Obstacle> quadrantObstacle[4];
                std::shared_ptr<Obstacle> goalObstacle[2];
        };
    }
}
