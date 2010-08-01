#pragma once

#include "Behavior.hpp"

// This macro lets PlayConfigTab automagically populate its list of available plays.
// It creates a PlayFactor for the given play class.
#define REGISTER_PLAY(x) static Gameplay::PlayFactory<x> __factory;
#define REGISTER_PLAY_CATEGORY(x, c) static Gameplay::PlayFactory<x> __factory(c);

namespace Gameplay
{
	class GameplayModule;
	
	class Play: public Behavior
	{
	public:
		// Create the play, with a given number of robots
		Play(GameplayModule *gameplay, size_t minRobots = 0);
		
		// Returns true iff this play is allowed to be selected given the current state of the game.
		// The default implementation always returns true.
		virtual bool applicable();
		
		// Returns a score used to compare this play against all other applicable plays when a new play
		// is to be selected.
		//
		// The applicable play with the LOWEST score is selected.  This is intended to be convenient for error-minimizing criteria.
		// The default implementation always returns zero.
		virtual float score();
		
		// If true, this play can be examined by GameplayModule::selectPlay.
		// The GUI thread should still use the enable/disable functions in GameplayModule because they are thread-safe.
		// The GUI thread may read this field because the processing field will never change it.
		bool enabled;
	};
	
	// The list of factories has to hold a single (and thus non-templated) type, so we use this base class
	// to add factories to the list of factories.
	class PlayFactoryBase
	{
		public:
			PlayFactoryBase(QString c = QString());
			
			virtual Play *create(GameplayModule *gameplay) = 0;
			
			QString category;
			
			// This has to be a pointer to a list because a static list may not be constructed before the factories
			static std::list<PlayFactoryBase *> *factories;
	};
	
	// This class is used to create a particular play.  It is created by REGISTER_PLAY and added to the
	// factory list by PlayFactoryBase's constructor.
	template<class X>
	class PlayFactory: public PlayFactoryBase
	{
		public:
			PlayFactory(QString c = QString()):
				PlayFactoryBase(c)
			{
			}
			
			virtual Play *create(GameplayModule *gameplay)
			{
				return new X(gameplay);
			}
	};
}
