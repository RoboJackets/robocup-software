/// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
/// vim:ai ts=4 et

#pragma once

#include <Geometry2d/Point.hpp>
#include <framework/Obstacle.hpp>

#include <set>
#include <QMutex>
#include <QString>

#include <boost/shared_ptr.hpp>
#include <boost/ptr_container/ptr_vector.hpp>

class OurRobot;
class SystemState;
class Configuration;

namespace Gameplay
{
	class Play;
	class PlayFactory;
	
	namespace Behaviors
	{
		class Goalie;
	}
	
	class GameplayModule
	{
		public:
			GameplayModule(SystemState *state);
			virtual ~GameplayModule();
			
			SystemState *state() const
			{
				return _state;
			}
			
			int manualID() const;
			
			void createGoalie();
			void removeGoalie();
			
			Behaviors::Goalie *goalie() const
			{
				return _goalie;
			}
			
			virtual void run();
			
			////////////
			// Useful matrices:
			// Each of these converts coordinates in some other system to team space.
			// Use them in a play or behavior like this:
			//   team = _gameplay->oppMatrix() * Geometry2d::Point(1, 0);
			
			/// Centered on the ball
			Geometry2d::TransformMatrix ballMatrix() const
			{
				return _ballMatrix;
			}
			
			/// Center of the field
			Geometry2d::TransformMatrix centerMatrix() const
			{
				return _centerMatrix;
			}
			
			/// Opponent's coordinates
			Geometry2d::TransformMatrix oppMatrix() const
			{
				return _oppMatrix;
			}
			
			/// Returns the name of the current play
			QString playName()
			{
				return _playName;
			}
			
			/// All robots on our team that are usable by plays
			const std::set<OurRobot *> &playRobots() const
			{
				return _playRobots;
			}


			void clearAvoidBallRadii();

			void goalieID(int value)
			{
				_goalieID = value;
			}
			int goalieID()
			{
				return _goalieID;
			}

			
		private:
			friend class Play;
			
			/// This protects all of Gameplay.
			/// This is held while plays are running.
			QMutex _mutex;
			
			SystemState *_state;
			
			Configuration *_config;

			/// The goalie behavior (may be null)
			Behaviors::Goalie *_goalie;
			
			std::set<OurRobot *> _playRobots;
			
			/// The current play
			std::shared_ptr<Play> _currentPlay;
			
			/// Factory which produced the current play
			PlayFactory *_currentPlayFactory;
			
			/// True if the current play is finished and a new one should be selected during the next frame
			bool _playDone;
			
			Geometry2d::TransformMatrix _ballMatrix;
			Geometry2d::TransformMatrix _centerMatrix;
			Geometry2d::TransformMatrix _oppMatrix;
			
			/// Obstacles to prevent using half the field
			std::shared_ptr<PolygonObstacle> _ourHalf;
			std::shared_ptr<PolygonObstacle> _opponentHalf;
			
			ObstaclePtr _sideObstacle;
			
			///outside of the floor boundaries
			ObstaclePtr _nonFloor[4];
			
			///goal area
			ObstacleGroup _goalArea;
			
			/// Name of the current play
			QString _playName;

			/// utility functions

			/**
			 * Checks the current play to determine if it is necessary to find a new one,
			 * and performs necessary updates
			 */
			void updatePlay();

			/**
			 * Returns the current set of global obstacles, including the field
			 */
			ObstacleGroup globalObstacles() const;

			int _our_score_last_frame;

			// Board ID of the robot to assign the goalie position
			int _goalieID;

	};
}
