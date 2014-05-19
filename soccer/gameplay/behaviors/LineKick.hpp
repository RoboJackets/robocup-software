#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/tactics/ChipCalibration.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class LineKick: public SingleRobotBehavior
		{
			public:
				static void createConfiguration(Configuration *cfg);

				LineKick(GameplayModule *gameplay);
				
				virtual bool run();
				
				bool done() const
				{
					return _state == State_Done;
				}
				
				void restart();
				
				Geometry2d::Point target;
				
				/** kick parameter flags */
				bool use_chipper;
				uint8_t kick_power;
				bool kick_ready;
				bool enable_kick;

			private:
				enum
				{
					State_Setup,
					State_Charge,
					State_Done
				} _state;

				bool ballClose;



				static ConfigDouble *_drive_around_dist;
				static ConfigDouble *_setup_to_charge_thresh;
				static ConfigDouble *_escape_charge_thresh;
				static ConfigDouble *_setup_ball_avoid;
				static ConfigDouble *_accel_bias;
				static ConfigDouble *_facing_thresh;
				static ConfigDouble *_max_speed;
				static ConfigDouble *_proj_time;
				static ConfigDouble *_dampening;
				static ConfigDouble *_done_thresh;
				static ConfigBool *_land_on_target;

				ChipCalibration chip_calib;
		};
	}
}
