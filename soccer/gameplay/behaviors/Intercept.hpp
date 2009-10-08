#pragma once

#include "../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Intercept: public Behavior
		{
			public:
				Intercept(GameplayModule *gameplay);

				virtual void assign(std::set<Robot *> &available);
				virtual bool run();

				Geometry2d::Point target;
				
			protected:
				virtual float score(Robot * robot);
				
				typedef enum
				{
					ApproachFar, 
					ApproachBall, 
					Done
				} State;

				State _state;
				
		};
	}
}
