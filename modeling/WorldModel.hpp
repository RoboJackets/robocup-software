#ifndef WORLD_MODEL_HPP_
#define WORLD_MODEL_HPP_

#include "framework/Module.hpp"
#include "RobotWM.hpp"
//#include "Ball.hpp"


/** World modeling system */
namespace Modeling
{
        class WorldModel : public Module
	{
		public:
			WorldModel();
			
			virtual void run();
			
		private:
			RobotWM _self[5];
			RobotWM _opp[5];
	  		//Ball* ball;
	};
}

#endif /* WORLD_MODEL_HPP_ */
