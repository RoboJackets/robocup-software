#pragma once

#include <boost/array.hpp>

#include <Constants.hpp>
#include <motion/RobotController.hpp>

class Configuration;

namespace Motion {

	/**
	 * Controller that converts a target point into wheel velocities on a
	 * robot-by-robot basis.  Optional flags change how we handle different cases
	 *
	 * The gameplay planner is responsible for pathfinding and general obstacle avoidance,
	 * as this controller will do nothing more than try to stop if there is a possible
	 * collision.
	 */
	class PointControlModule {

	public:

		PointControlModule(SystemState *state, Configuration *cfg);
		~PointControlModule() {}

		/** executes the module's commands for the frame */
		void run();

	private:
		SystemState *_state;

		Configuration *_config;

		boost::array<Motion::RobotController::shared_ptr, Num_Shells> _robots;
	};

}

