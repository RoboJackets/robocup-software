
#pragma once

//	note: for an odd Qt-related issue, this python include has to come before
//			the Qt includes (because of the 'slots' macro)
#include <boost/python.hpp>

#include <Geometry2d/TransformMatrix.hpp>
#include <Geometry2d/Polygon.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/CompositeShape.hpp>

#include <set>
#include <QMutex>
#include <QString>

#include <boost/ptr_container/ptr_vector.hpp>

class OurRobot;
class SystemState;


/**
 * @brief Higher-level logic for soccer
 * @details The Gameplay namespace contains things like Plays, Behaviors, Tactics
 * and the GameplayModule.
 */
namespace Gameplay
{
	/**
	 * @brief Coordinator of high-level logic
	 * 
	 * @details Its main responsibilities include:
	 * - managing the Goalie
	 * - maintaining a list of global field obstacles
	 * - choosing which Play to run
	 * - running the current play
	 * - executing path planning for each OurRobot
	 */
	class GameplayModule
	{
		public:
			GameplayModule(SystemState *state);
			virtual ~GameplayModule();
			
			SystemState *state() const
			{
				return _state;
			}
			
			virtual void run();
			
			void setupUI();

			void goalieID(int value);
			int goalieID()
			{
				return _goalieID;
			}
			

			/**
			 * @defgroup matrices Coordinate Conversion Matrices
			 * Each of these matrices converts coordinates from some other system
			 * to team space.
			 * 
			 * Example:
			 * team = _gameplay->oppMatrix() * Geometry2d::Point(1, 0);
			 */

			
			/**
			 * Centered on the ball
			 * @ingroup matrices
			 */
			Geometry2d::TransformMatrix ballMatrix() const
			{
				return _ballMatrix;
			}
			
			/**
			 * Center of the field
			 * @ingroup matrices
			 */
			Geometry2d::TransformMatrix centerMatrix() const
			{
				return _centerMatrix;
			}
			
			/**
			 * Opponent's coordinates
			 * @ingroup matrices
			 */
			Geometry2d::TransformMatrix oppMatrix() const
			{
				return _oppMatrix;
			}
			
			/// All robots on our team that are usable by plays
			const std::set<OurRobot *> &playRobots() const
			{
				return _playRobots;
			}


		protected:

			boost::python::object getRootPlay();

			///	gets the instance of the main.py module that's loaded at GameplayModule
			boost::python::object getMainModule();

			
		private:
			/// This protects all of Gameplay.
			/// This is held while plays are running.
			QMutex _mutex;
			
			SystemState *_state;
			
			std::set<OurRobot *> _playRobots;
			
			Geometry2d::TransformMatrix _ballMatrix;
			Geometry2d::TransformMatrix _centerMatrix;
			Geometry2d::TransformMatrix _oppMatrix;
			
			/// Obstacles to prevent using half the field
			std::shared_ptr<Geometry2d::Polygon> _ourHalf;
			std::shared_ptr<Geometry2d::Polygon> _opponentHalf;
			
			std::shared_ptr<Geometry2d::Shape> _sideObstacle;
			
			///	outside of the floor boundaries
			std::shared_ptr<Geometry2d::Shape> _nonFloor[4];
			
			///	goal area
			Geometry2d::CompositeShape _goalArea;

			/// utility functions

			/**
			 * Returns the current set of global obstacles, including the field
			 */
			Geometry2d::CompositeShape globalObstacles() const;

			int _our_score_last_frame;

			// Shell ID of the robot to assign the goalie position
			int _goalieID;


			//	python
			boost::python::object _mainPyNamespace;
	};
}
