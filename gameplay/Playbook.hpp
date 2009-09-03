// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <string>
#include <list>

namespace Gameplay
{
	class Play;
	class Behavior;
	class GameplayModule;
	
	class Playbook
	{
	public:
		Playbook(GameplayModule *gameplay);
		
		// Loads all .play files from the given directory.
		void loadDir(const std::string &path);
		
		// Loads a single play from a file.
		void load(const std::string &path);

		// Selects a goalie and a play if necessary.
		// Returns true if a new play has been selected.
		bool setup();
		
		// Runs the goalie and the current play.
		void run();
		
		// Aborts the current play.
		// A new play will be selected when run() is called.
		void abort();

		// Sets the goalie behavior.
		// The goalie always has top priority when assigning robots.
		// If any robots are available, the goalie will be assigned one.
		void goalie(Behavior *goalie);
		
		Behavior *goalie() const
		{
			return _goalie;
		}

		// Selects a new play.
		Play *selectPlay();

		Play *currentPlay() const { return _currentPlay; }
		const std::list<Play *> &plays() const { return _plays; }

	protected:
		GameplayModule *_gameplay;
		
		// Used for sorting plays.
		bool comparePlays(Play *p1, Play *p2);
		
		Behavior *_goalie;
		Play *_currentPlay;
		
		// True if the current play has just been selected and needs to be started in run().
		bool _needStart;
		
		std::list<Play *> _plays;
		
		// True when the reason the current play ended has been printed.
		bool _logged_end;
	};
}
