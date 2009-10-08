#pragma once

#include "../Behavior.hpp"
#include "../parameters/Point_Parameter.hpp"
#include "../parameters/Float_Parameter.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class Move: public Behavior
        {
        public:
            Move(GameplayModule *gameplay, Role *role);
            
            virtual void run();
            virtual bool done();
            
            virtual float score(Robot* robot);
            
        protected:
            Point_Parameter pos_param;
            Point_Parameter face_param;
            Float_Parameter threshold_param;
            Float_Parameter backoff_param;
        };
    }
}
