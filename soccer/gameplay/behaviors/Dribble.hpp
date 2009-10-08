#pragma once

#include "../Behavior.hpp"

#include "../parameters/Point_Parameter.hpp"

#include "Intercept.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Dribble: public Behavior
		{
			public:
				Dribble(GameplayModule *gameplay, Role *role);
				
				virtual void run();
				virtual void start();
				virtual bool done();
				
				virtual float score(Robot* robot);
				
			protected:
				typedef enum
				{
					AcquireBall,
					Translate
				} State;
				
				//optional parameter
				Point_Parameter _dest;
				
				State _state;
				
				Intercept _intercept;
		};
	}
}
