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
			WorldModel(SystemState *state, const ConfigFile::WorldModel& cfg);
			~WorldModel();

			virtual void run();

		protected:
			SystemState *_state;

			// Add to opponents' shell IDs to get track map keys.
			static const int OppOffset = 256;

			// track all observed robots
			typedef std::map<int, RobotModel::shared> RobotMap;
			RobotMap _robotMap;

			BallModel ballModel;

			// keep track of which robot is in which slot
			std::vector<int> _selfSlots;
			std::vector<int> _oppSlots;

			const ConfigFile::WorldModel& _config;
	};
}
