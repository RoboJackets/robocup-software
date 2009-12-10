/*
 * PassOptimizer.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Alex Cunningham
 */

#pragma once

#include "../GameplayModule.hpp"

namespace Gameplay
{
	namespace Optimization
	{
		class PassOptimizer {

		public:
			PassOptimizer(GameplayModule* gameplay);
			virtual ~PassOptimizer();

		protected:
			GameplayModule * gameplay_;

		};
	}
}

