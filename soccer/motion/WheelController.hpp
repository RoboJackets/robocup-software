#pragma once

#include <boost/array.hpp>

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

	private:
		/** information pertaining to a single robot axle */
		typedef struct Axle
		{
			Axle(const Geometry2d::Point axel = Geometry2d::Point())
			{
				motor = 0;
				lastWheelVel = 0;
				wheel = axel.perpCCW();
			}

			//motor value
			float motor;

			//weeel velocities in the last frame
			int8_t lastWheelVel;

			//axle vector
			Geometry2d::Point axle;
			Geometry2d::Point wheel;
		} Axle;

		typedef boost::array<Axle, 4> Axles;
		static Axles initAxles(const QVector<Geometry2d::Point>& points);

	public:

		typedef boost::array<int8_t, 4> MotorCmd;

		WheelController(SystemState *state, const ConfigFile::MotionModule& cfg);
		~WheelController() {}

		void run();

	private:
		SystemState *_state;

		const ConfigFile::MotionModule& _config;

		/**
		 * function that actually generates motor commands
		 * Needs velocity as well as a dynamics model
		 */
		MotorCmd genMotor(const Geometry2d::Point& vel, float w, const SystemState::Robot& robot);
	};

}

