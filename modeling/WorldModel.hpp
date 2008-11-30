#ifndef WORLD_MODEL_HPP_
#define WORLD_MODEL_HPP_

#include "framework/Module.hpp"

/** World modeling system */
namespace Modeling
{
	class WorldModel : public Module
	{
		public:
			WorldModel();
			
			virtual void run();
			
		private:
			/** buffer previous states */
			
			/** Kalman Filters for robots */
			
			/** Kalman Filters for ball estimates */
	};
}

#endif /* WORLD_MODEL_HPP_ */