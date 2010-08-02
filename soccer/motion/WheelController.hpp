#pragma once

#include <Constants.hpp>
#include <framework/SystemState.hpp>
#include <framework/ConfigFile.hpp>

namespace Motion {

	/**
	 * Controller that converts motion velocities to
	 * wheel velocities. For now, this is relatively simple,
	 * but could become more complex to handle traction
	 * control scenarios.
	 */
	class WheelController {
	public:
		WheelController(SystemState *state, const ConfigFile::MotionModule& cfg);
		~WheelController() {}

		void run();

	private:
		SystemState *_state;

		const ConfigFile::MotionModule& _config;
	};

}

