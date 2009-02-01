#ifndef WORLD_MODEL_HPP_
#define WORLD_MODEL_HPP_

#include "framework/Module.hpp"
//#include "Robot.hpp"
#include "Ball.hpp"

#include <vector>

/** World modeling system */
namespace Modeling
{
        class WorldModel : public Module
	{
		public:
			WorldModel();
			
			virtual void run();
			
		private:
// 	                std::vector<Modeling::Robot*>* blueTeam;
// 	                std::vector<Modeling::Robot*>* yellowTeam;
	  		Ball* ball;
	};
}

#endif /* WORLD_MODEL_HPP_ */
