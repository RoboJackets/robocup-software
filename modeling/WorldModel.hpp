#ifndef WORLD_MODEL_HPP_
#define WORLD_MODEL_HPP_

#include <QString>
#include <framework/Module.hpp>
#include "RobotWM.hpp"
#include <config/ConfigFile.hpp>
//#include "Ball.hpp"


/** World modeling system */
namespace Modeling
{
        class WorldModel : public Module
	{
		public:
			WorldModel(QString filename);
			
			virtual void run();
			
		private:
			RobotWM _self[5];
			RobotWM _opp[5];
	  		//Ball* ball;

	  		ConfigFile _config;
	};
}

#endif /* WORLD_MODEL_HPP_ */
