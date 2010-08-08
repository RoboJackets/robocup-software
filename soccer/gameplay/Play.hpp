#pragma once

#include "Behavior.hpp"

// This macro lets PlayConfigTab automagically populate its list of available plays.
// It creates a PlayFactory for the given play class.
#define REGISTER_PLAY(x) static Gameplay::PlayFactoryImpl<x> __factory;
#define REGISTER_PLAY_CATEGORY(x, c) static Gameplay::PlayFactoryImpl<x> __factory(c);

namespace Gameplay
{
	class GameplayModule;
	
	class Play: public Behavior
	{
	public:
		Play(GameplayModule *gameplay);
		
		// Every subclass of Play needs to override this function.
		// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
		static float score(GameplayModule *gameplay);
	};
	
	// The list of factories has to hold a single (and thus non-templated) type, so we use this base class
	// to add factories to the list of factories.
	class PlayFactory
	{
		public:
			PlayFactory(QString c = QString());
			
			virtual float score(GameplayModule *gameplay) = 0;
			virtual Play *create(GameplayModule *gameplay) = 0;
			virtual QString name() = 0;
			
			QString category;
			
			bool enabled;
			
			// Cached score() value from last gameplay iteration
			float lastScore;
			
			static const std::list<PlayFactory *> &factories();
			
		protected:
			// This has to be a pointer to a list because a static list may not be constructed before the factories
			static std::list<PlayFactory *> *_factories;
	};
	
	// This class is used to create a particular play.  It is created by REGISTER_PLAY and added to the
	// factory list by PlayFactoryBase's constructor.
	template<class X>
	class PlayFactoryImpl: public PlayFactory
	{
		public:
			PlayFactoryImpl(QString c = QString()):
				PlayFactory(c)
			{
			}
			
			virtual float score(GameplayModule *gameplay)
			{
				return X::score(gameplay);
			}
			
			virtual Play *create(GameplayModule *gameplay)
			{
				Play *play = new X(gameplay);
				return play;
			}
			
			virtual QString name()
			{
				if (_name.isNull())
				{
					_name = Utils::className(typeid(X));
				}
				
				return _name;
			}
			
		protected:
			QString _name;
	};
}
