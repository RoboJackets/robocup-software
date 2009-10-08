#pragma once

#include "Condition.hpp"

#include <string>
#include <list>
#include <QTime>

namespace Gameplay
{
	class Play;
	class Role;
	class Parameter;
	class GameplayModule;

	class Play
	{
	public:
		Play(GameplayModule *gameplay);
		~Play();
		
		GameplayModule *gameplay() const
		{
			return _gameplay;
		}
		
		void start();
		
		// Runs the play for one frame.
		// Returns true if the play can continue running or false if another play should be selected.
		bool run();
		void stop();
		
	protected:
		friend class Role;
		
		GameplayModule *_gameplay;
		std::list<Role *> _roles;
	};
}
