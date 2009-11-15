/*
 * Receiver.hpp
 *
 *  Created on: Nov 8, 2009
 *      Author: alexgc
 */


#pragma once

#include "../../Behavior.hpp"

namespace Gameplay
{
	namespace Behaviors
	{
		class Receiver: public Behavior
		{
			public:
				Receiver(GameplayModule *gameplay);

				virtual bool run();
				virtual bool done();

				virtual float score(Robot* robot);
		};
	}
}

