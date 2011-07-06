// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <gameplay/GameplayModule.hpp>
#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/positions/Goalie.hpp>
#include <gameplay/Play.hpp>
#include <Constants.hpp>
#include <protobuf/LogFrame.pb.h>
#include <Robot.hpp>

#include <stdio.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace boost;

Gameplay::GameplayModule::GameplayModule(SystemState *state):
	_mutex(QMutex::Recursive)
{
	_state = state;
	_goalie = 0;
	_currentPlayFactory = 0;
	_playDone = false;

	_centerMatrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Field_Length / 2));
	_oppMatrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Field_Length)) *
				Geometry2d::TransformMatrix::rotate(180);

	// Make an obstacle to cover the opponent's half of the field except for one robot diameter across the center line.
	PolygonObstacle *sidePolygon = new PolygonObstacle;
	_sideObstacle = ObstaclePtr(sidePolygon);
	float x = Field_Width / 2 + Field_Border;
	const float y1 = Field_Length / 2;
	const float y2 = Field_Length + Field_Border;
	const float r = Field_CenterRadius;
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(-x, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(-r, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(0, y1 + r));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(r, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(x, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(x, y2));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(-x, y2));

	float y = -Field_Border;
	float deadspace = Field_Border;
	x = Floor_Width/2.0f;
	PolygonObstacle* floorObstacle = new PolygonObstacle;
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y-1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y-1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[0] = ObstaclePtr(floorObstacle);

	y = Field_Length + Field_Border;
	floorObstacle = new PolygonObstacle;
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y+1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y+1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[1] = ObstaclePtr(floorObstacle);

	y = Floor_Length;
	floorObstacle = new PolygonObstacle;
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, -deadspace));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x-1, -deadspace));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x-1, y));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y));
	_nonFloor[2] = ObstaclePtr(floorObstacle);

	floorObstacle = new PolygonObstacle;
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, -deadspace));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x+1, -deadspace));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x+1, y));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[3] = ObstaclePtr(floorObstacle);

	PolygonObstacle* goalArea = new PolygonObstacle;
	const float halfFlat = Field_GoalFlat/2.0;
	const float radius = Field_ArcRadius;
	goalArea->polygon.vertices.push_back(Geometry2d::Point(-halfFlat, 0));
	goalArea->polygon.vertices.push_back(Geometry2d::Point(-halfFlat, radius));
	goalArea->polygon.vertices.push_back(Geometry2d::Point( halfFlat, radius));
	goalArea->polygon.vertices.push_back(Geometry2d::Point( halfFlat, 0));
	_goalArea.add(ObstaclePtr(goalArea));
	_goalArea.add(ObstaclePtr(new CircleObstacle(Geometry2d::Point(-halfFlat, 0), radius)));
	_goalArea.add(ObstaclePtr(new CircleObstacle(Geometry2d::Point(halfFlat, 0), radius)));

	_ourHalf = make_shared<PolygonObstacle>();
	_ourHalf->polygon.vertices.push_back(Geometry2d::Point(-x, -Field_Border));
	_ourHalf->polygon.vertices.push_back(Geometry2d::Point(-x, y1));
	_ourHalf->polygon.vertices.push_back(Geometry2d::Point(x, y1));
	_ourHalf->polygon.vertices.push_back(Geometry2d::Point(x, -Field_Border));

	_opponentHalf = make_shared<PolygonObstacle>();
	_opponentHalf->polygon.vertices.push_back(Geometry2d::Point(-x, y1));
	_opponentHalf->polygon.vertices.push_back(Geometry2d::Point(-x, y2));
	_opponentHalf->polygon.vertices.push_back(Geometry2d::Point(x, y2));
	_opponentHalf->polygon.vertices.push_back(Geometry2d::Point(x, y1));
}

Gameplay::GameplayModule::~GameplayModule()
{
	removeGoalie();
}

int Gameplay::GameplayModule::manualID() const
{
	return _state->logFrame->manual_id();
}

void Gameplay::GameplayModule::createGoalie()
{
	if (!_goalie)
	{
		printf("create goalie\n");
  		_goalie = new Behaviors::Goalie(this);
	}
}

void Gameplay::GameplayModule::removeGoalie()
{
	if (_goalie)
	{
		delete _goalie;
		_goalie = 0;
	}
}

void Gameplay::GameplayModule::updatePlay() {
	const bool verbose = false;

	// important scenarios:
	//  - no current plays - must pick a new one
	//  - no available plays - must do nothing
	//  - current play is fine, must be run
	//  - current play has ended, must select new and run
	//  - current play is not applicable/failed, must kill, select new and run
	//  - new goalie, select new play (may reselect current play, which much be assigned new robots)

	// Update play scores.
	// The GUI thread needs this for the Plays tab and we will use it later
	// to determine the new play.
	BOOST_FOREACH(PlayFactory *factory, PlayFactory::factories())
	{
	factory->lastScore = factory->score(this);
	}

	if (	_playDone || 							// New play if old one was complete
			!_currentPlay ||						// There is no current play
			isinf(_currentPlayFactory->lastScore) || // Current play doesn't apply anymore
			!_currentPlayFactory->enabled			// Current play was disabled
	)
	{
		if (verbose) cout << "  Selecting a new play" << endl;
		_playDone = false;

		// Find the next play
		PlayFactory *bestPlay = 0;
		// Pick a new play
		float bestScore = 0;

		// Find the best applicable play
		BOOST_FOREACH(PlayFactory *factory, PlayFactory::factories())
		{
			if (factory->enabled)
			{
				float score = factory->lastScore;
				if (!isinf(score) && (!bestPlay || score < bestScore))
				{
					bestScore = score;
					bestPlay = factory;
				}
			}
		}

		// Start the play if it's not current.
		if (bestPlay)
		{
			if (bestPlay != _currentPlayFactory)
			{
				_currentPlayFactory = bestPlay;
				_currentPlay = shared_ptr<Play>(_currentPlayFactory->create(this));
			}
		} else {
			// No usable plays
			_currentPlay.reset();
			_currentPlayFactory = 0;
		}
	}
}

ObstacleGroup Gameplay::GameplayModule::globalObstacles() const {
	ObstacleGroup obstacles;
	if (_state->gameState.stayOnSide())
	{
		obstacles.add(_sideObstacle);
	}

	if (!_state->logFrame->use_our_half())
	{
		obstacles.add(_ourHalf);
	}

	if (!_state->logFrame->use_opponent_half())
	{
		obstacles.add(_opponentHalf);
	}

	// Add non floor obstacles
	BOOST_FOREACH(const ObstaclePtr& ptr, _nonFloor)
	{
		obstacles.add(ptr);
	}
	return obstacles;
}

void Gameplay::GameplayModule::run()
{
	QMutexLocker lock(&_mutex);
	
	bool verbose = false;
	if (verbose) cout << "Starting GameplayModule::run()" << endl;

	// perform state variable updates on robots
	// Currently - only the timer for the kicker charger
	BOOST_FOREACH(OurRobot* robot, _state->self)
	{
		if (robot) {
			robot->update();
			robot->resetMotionCommand();
		}
	}

	// Build a list of visible robots
	_playRobots.clear();
	BOOST_FOREACH(OurRobot *r, _state->self)
	{
		if (r->visible && !r->exclude)
		{
			_playRobots.insert(r);
		}
	}

	// Assign the goalie and remove it from _playRobots
	if (_goalie)
	{
		// The Goalie behavior is responsible for only selecting a robot which is allowed by the rules
		// (no changing goalies at random times).
		// The goalie behavior has priority for choosing robots because it must obey this rule,
		// so the current play will be restarted in case the goalie stole one of its robots.
		_goalie->assign(_playRobots);
	}
	
	if (_playRobots.size() == 5)
	{
		printf("Too many robots on field: goalie %p\n", _goalie);
		if (_goalie)
		{
			printf("  robot %p\n", _goalie->robot);
			if (_goalie->robot)
			{
				printf("  shell %d\n", _goalie->robot->shell());
			}
		}
		
		// Make a new goalie
		if (_goalie)
		{
			delete _goalie;
		}
  		_goalie = new Behaviors::Goalie(this);
	}

	_ballMatrix = Geometry2d::TransformMatrix::translate(_state->ball.pos);

	if (verbose) cout << "  Updating play" << endl;
	updatePlay();

	// Run the goalie
	if (_goalie)
	{
		if (verbose) cout << "  Running goalie" << endl;
		if (_goalie->robot && _goalie->robot->visible)
		{
			_goalie->run();
		}
	}

	// Run the current play
	if (_currentPlay)
	{
		if (verbose) cout << "  Running play" << endl;
		_playDone = !_currentPlay->run();
	}

	// determine global obstacles - field requirements
	// Two versions - one set with goal area, another without for goalie
	ObstacleGroup global_obstacles = globalObstacles();
	ObstacleGroup obstacles_with_goal = global_obstacles;
	obstacles_with_goal.add(_goalArea);

	// execute motion planning for each robot
	BOOST_FOREACH(OurRobot* r, _state->self) {
		if (r && r->visible) {
			// set obstacles for the robots
			if (_goalie && _goalie->robot && r->shell() == _goalie->robot->shell())
				r->execute(global_obstacles); // just for goalie
			else
				r->execute(obstacles_with_goal); // all other robots
		}
	}

	// visualize
	if (_state->gameState.stayAwayFromBall() && _state->ball.valid)
	{
		_state->drawCircle(_state->ball.pos, Field_CenterRadius, Qt::black, "Rules");
	}

	if (_currentPlay)
	{
		_playName = _currentPlay->name();
	} else {
		_playName = QString();
	}
	if (verbose) cout << "Finishing GameplayModule::run()" << endl;
}
