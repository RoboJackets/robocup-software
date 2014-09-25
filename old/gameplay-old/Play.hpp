#pragma once

#include <AutoName.hpp>
#include "Behavior.hpp"

namespace Gameplay
{
	class GameplayModule;
	/**
	 * base class for implementing actual plays
	 * includes functions to assist in finding robots
	 */
	class Play: public Behavior, public AutoName
	{
	public:
		Play(GameplayModule *gameplay);
		
		/// Every subclass of Play needs to override this function.
		/// Return INFINITY if the play cannot be used or a score (lower is better) used to select the best play.
		static float score(GameplayModule *gameplay);
	};
	
	/// The list of factories has to hold a single (and thus non-templated) type, so we use this base class
	/// to add factories to the list of factories.
	class PlayFactory
	{
		public:
			PlayFactory(QString c = QString());
			
			virtual float score(GameplayModule *gameplay) = 0;
			virtual Play *create(GameplayModule *gameplay) = 0;
			virtual QString name() = 0;
			
			QString category;
			
			bool enabled;
			
			/// Cached score() value from last gameplay iteration
			float lastScore;
			
			static const std::list<PlayFactory *> &factories();
			
		protected:
			/// This has to be a pointer to a list because a static list may not be constructed before the factories
			static std::list<PlayFactory *> *_factories;
	};
	
	/// This class is used to create a particular play.  It is created by REGISTER_PLAY and added to the
	/// factory list by PlayFactoryBase's constructor.
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
					_name = className(typeid(X));
				}
				
				return _name;
			}
			
		protected:
			QString _name;
	};
}

////////////
// Assignment functions
//
// These are used to find the best robot according to some criteria,
// remove it from the available set, and store it in role.  The capabilities
// structure provides a set of requirements a robot must meet, such as
// having a chipper, or the kicker being charged. Each robot will know its
// capabilities.  Visibility is now included in requirements.
//
// If needVisible is true and the currently assigned robot is not visible,
// a new robot will be selected.  If no robot can be selected, role
// is unchanged and the function returns false.
//
/// Each assigner returns true iff the role has a usable robot.

/// Assigns the robot in nearest to <pt>
/// places no constraints other than visibility
bool assignNearest(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt);

/// require kick/chip/dribble/sense
bool assignNearestFull(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt);

/// Assigns the nearest robot that can dribble and kick
bool assignNearestKicker(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt);

/// Assigns the nearest robot that can chip
bool assignNearestChipper(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt);

/// Assigns the nearest robot that can yank
bool assignNearestYank(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt);

/// General "AssignNearest" with a full set of constraints - true requires that it be met
bool assignNearest(OurRobot *&role, std::set<OurRobot *> &robots, Geometry2d::Point pt,
		bool hasKicker, bool hasChipper, bool hasDribbler, bool hasBallSense);
