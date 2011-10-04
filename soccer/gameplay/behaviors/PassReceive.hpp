#pragma once

#include "../Behavior.hpp"
#include "Kick.hpp"

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
				bool done();

				bool passUseChip;
				bool passUseLine;

				uint8_t passPower;
				uint64_t backupTime;

			protected:
				Behaviors::Kick _passer;

				static ConfigInt *_backup_time;
				static ConfigDouble *_backup_speed;



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
