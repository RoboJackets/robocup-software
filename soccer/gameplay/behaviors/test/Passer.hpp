/*
 * Passer.hpp
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
		class Passer: public Behavior
		{
			public:
				Passer(GameplayModule *gameplay);

				virtual bool run();

				virtual float score(Robot* robot);
		};
	}
}
