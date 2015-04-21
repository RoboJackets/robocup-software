#pragma once

#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Behaviors
	{
		class PassReceive: public TwoRobotBehavior
		{
			public:
				static void createConfiguration(Configuration *cfg);
				PassReceive(GameplayModule *gameplay);
				
				virtual bool run();
				void reset();
				void setEnable(bool b);
				bool done();
				bool kicked();
				uint8_t calcKickPower(int d);

				bool passUseChip;
				bool passUseLine;

				uint8_t passPower;
				uint64_t backupTime;

				bool enable;

			protected:
				Behaviors::Kick _passer;

				static ConfigInt *_backup_time;
				static ConfigDouble *_backup_speed;
				static ConfigDouble *_done_thresh;
				static ConfigDouble *_kick_power_constant;



			private:
				uint64_t readyTime;
				enum
				{
					State_Setup,
					State_Ready,
					State_Execute,
					State_Kicked,
					State_Done
				} _state;


		};
	}
}
