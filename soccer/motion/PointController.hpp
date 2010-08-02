#pragma once

#include <boost/array.hpp>

#include <Constants.hpp>
#include <framework/ConfigFile.hpp>
#include <motion/Robot.hpp>

namespace Motion {

	/**
	 * Controller that converts a target point into wheel velocities on a
	 * robot-by-robot basis.  Optional flags change how we handle different cases
	 *
	 * The gameplay planner is responsible for pathfinding and general obstacle avoidance,
	 * as this controller will do nothing more than try to stop if there is a possible
	 * collision.
	 */
	class PointController {

	public:

		PointController(SystemState *state, const ConfigFile::MotionModule& cfg);
		~PointController() {}

		/** executes the module's commands for the frame */
		void run();

	private:
		SystemState *_state;

		const ConfigFile::MotionModule& _config;

		boost::array<Motion::Robot::shared_ptr, Constants::Robots_Per_Team> _robots;
	};

}

