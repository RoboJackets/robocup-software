#pragma once

#include "../Behavior.hpp"

#include "../parameters/Robot_Parameter.hpp"
#include "Intercept.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Intercept;
	  
		class Steal: public Behavior
		{
			public:
				Steal(GameplayModule *gameplay, Role *role = 0);
				
				virtual void start();
				virtual void run();
				virtual bool done();
				
				virtual float score(Robot* robot);
				
			protected:
				typedef enum
				{
					Intercept,
					Maneuver,
					Stealing,
					Done
				} State;
				
                Gameplay::Behaviors::Intercept _intercept;

				State _state;
				float _lastMargin;
				Geometry2d::Segment _target;
				
				//we lock in a pivot point
				Geometry2d::Point _pivot;
				
				// The robot's position when it entered the Shoot state
				Geometry2d::Point _shootStart;
		  
		};
	}
}
