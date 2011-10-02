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
                ObstaclePtr centerObstacle[2];
                ObstaclePtr quadrantObstacle[4];
                ObstaclePtr goalObstacle[2];
        };
    }
}
