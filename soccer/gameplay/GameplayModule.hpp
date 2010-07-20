// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#pragma once

#include <QMutex>
#include <set>

#include <framework/ConfigFile.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

#include <gameplay/Robot.hpp>

namespace Gameplay
{
	class Behavior;
	class Play;
	
	class GameplayModule
	{
		public:
			GameplayModule(SystemState *state, const ConfigFile::MotionModule& cfg);
			~GameplayModule();
			
			SystemState *state() const
			{
				return _state;
			}
			
			void createGoalie();
			void removeGoalie();
			
			Behavior *goalie() const
			{
				return _goalie;
			}
			
			boost::shared_ptr<Play> currentPlay() const
			{
				return _currentPlay;
			}
			
			virtual void run();
			
			////////
			// Useful matrices:
			// Each of these converts coordinates in some other system to team space.
			// Use them in a play or behavior like this:
			//   team = _gameplay->oppMatrix() * Geometry2d::Point(1, 0);
			
			// Centered on the ball
			Geometry2d::TransformMatrix ballMatrix() const
			{
				return _ballMatrix;
			}
			
			// Center of the field
			Geometry2d::TransformMatrix centerMatrix() const
			{
				return _centerMatrix;
			}
			
			// Opponent's coordinates
			Geometry2d::TransformMatrix oppMatrix() const
			{
				return _oppMatrix;
			}
			
			void enablePlay(boost::shared_ptr<Play> play);
			void disablePlay(boost::shared_ptr<Play> play);
			bool playEnabled(boost::shared_ptr<Play> play);
			
			// Returns the name of the current play
			QString playName()
			{
				return _playName;
			}
			
			// This may be used by the GUI thread because the processing thread
			// won't change the list of plays.
			const std::set<boost::shared_ptr<Play> > &plays() const
			{
				return _plays;
			}

			Robot *self[Constants::Robots_Per_Team];
			Robot *opp[Constants::Robots_Per_Team];
			
		private:
			friend class Play;
			
			boost::shared_ptr<Play> selectPlay(size_t nrRobots);
			
			SystemState *_state;
			
			// The goalie behavior (may be null)
			Behavior *_goalie;
			
			// The current play
			boost::shared_ptr<Play> _currentPlay;
			
			// True if the current play is finished and a new one should be selected during the next frame
			bool _playDone;
			
			Geometry2d::TransformMatrix _ballMatrix;
			Geometry2d::TransformMatrix _centerMatrix;
			Geometry2d::TransformMatrix _oppMatrix;
			
			ObstaclePtr _sideObstacle;
			
			//outside of the floor boundaries
			ObstaclePtr _nonFloor[4];
			
			//goal area
			ObstaclePtr _goalArea[3];
			
			QMutex _playMutex;
			std::set<boost::shared_ptr<Play> > _plays;
			
			// Name of the current play
			QString _playName;

			// motion config information
			const ConfigFile::MotionModule& _motion_config;
	};
}
