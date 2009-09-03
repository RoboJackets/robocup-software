#pragma once

#include "../Behavior.hpp"

#include "../parameters/Point_Parameter.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class ChangeMe: public Behavior
		{
			public:
				ChangeMe(GameplayModule *gameplay, Role *role);
				
				virtual void run();
				virtual bool done();
				
				virtual float score(Robot* robot);
				
			protected:
				//optional parameter
				Point_Parameter pos_param;
		};
	}
}
