// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#pragma once

/** Semantic State Identification system */
namespace StateIdentification
{
	class StateIDModule
	{
		public:
			enum Possession
			{
				OFFENSE = 0,
				DEFENSE = 1,
				FREEBALL = 2
			};
			
			enum BallFieldPos
			{
				HOMEFIELD = 0,
				MIDFIELD = 1,
				OPPFIELD = 2
			};
			
			class GameStateID
			{
				public:
					Possession posession;
					BallFieldPos field_pos;
					
					GameStateID()
					{
						posession = OFFENSE;
						field_pos = HOMEFIELD;
					}
			};
			
			StateIDModule(SystemState *state);
			~StateIDModule();
			
			void run();
		
		protected:
			SystemState *_state;

			// Utility functions for each of the general parameters

			/** Generates a new possession state from the current one */
			SystemState::Possession updatePossession(const SystemState::Possession& cur_state);

			/** Determines the ball field position - with hysteresis */
			SystemState::BallFieldPos updateFieldPos(const SystemState::BallFieldPos& cur_pos);
	};
}
