// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

#include <framework/Module.hpp>
#include <framework/ConfigFile.hpp>

#include <QString>
#include <vector>
#include <list>
#include <map>

#include "BallModel.hpp"
#include "RobotModel.hpp"

/** World modeling system */
namespace Modeling
{
	class WorldModel : public Module
	{
		public:
			typedef boost::shared_ptr<WorldModel> shared_ptr;
			WorldModel(SystemState *state, ConfigFile::WorldModel& cfg);
			~WorldModel();
			
			virtual void run();
			
		protected:
			SystemState *_state;
			
			// Add to opponents' shell IDs to get track map keys.
			static const int OppOffset = 256;
			
			typedef std::map<int, RobotModel *> RobotMap;
			RobotMap _robotMap;
			
			BallModel ballModel;
			RobotModel *_selfRobot[5];
			RobotModel *_oppRobot[5];
			
			ConfigFile::WorldModel& _config;
	};
}
