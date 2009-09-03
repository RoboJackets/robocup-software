#pragma once

#pragma once

#include "../../Behavior.hpp"

namespace Gameplay
{
    namespace Behaviors
    {
        class Pivot: public Behavior
        {
        	typedef enum
        	{
        		ApproachFast,
        		ApproachSlow,
        		PivotAround
        	} State;
        	
			public:
				Pivot(GameplayModule *gameplay, Role *role);
				
				virtual void run();
				virtual void stop();
				virtual bool done();
				
			protected:
				State _state;
				Geometry2d::Point _pp;
        };
    }
}
