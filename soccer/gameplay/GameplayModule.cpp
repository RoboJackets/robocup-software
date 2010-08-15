// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <gameplay/GameplayModule.hpp>
#include <gameplay/Behavior.hpp>
#include <gameplay/behaviors/positions/Goalie.hpp>
#include <gameplay/Play.hpp>
#include <Constants.hpp>

#include <stdio.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost;

Gameplay::GameplayModule::GameplayModule(SystemState *state):
	_mutex(QMutex::Recursive)
{
	_state = state;
	_goalie = 0;
	_currentPlayFactory = 0;
	_playDone = false;

	_centerMatrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Constants::Field::Length / 2));
	_oppMatrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Constants::Field::Length)) *
				Geometry2d::TransformMatrix::rotate(180);

	// Make an obstacle to cover the opponent's half of the field except for one robot diameter across the center line.
	PolygonObstacle *sidePolygon = new PolygonObstacle;
	_sideObstacle = ObstaclePtr(sidePolygon);
	float x = Constants::Field::Width / 2 + Constants::Field::Border;
	const float y1 = Constants::Field::Length / 2;
	const float y2 = Constants::Field::Length + Constants::Field::Border;
	const float r = Constants::Field::CenterRadius;
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(-x, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(-r, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(0, y1 + r));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(r, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(x, y1));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(x, y2));
	sidePolygon->polygon.vertices.push_back(Geometry2d::Point(-x, y2));

	float y = -Constants::Field::Border;
	float deadspace = Constants::Field::Border;
	x = Constants::Floor::Width/2.0f;
	PolygonObstacle* floorObstacle = new PolygonObstacle;
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y-1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y-1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[0] = ObstaclePtr(floorObstacle);

	y = Constants::Field::Length + Constants::Field::Border;
	floorObstacle = new PolygonObstacle;
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(-x, y+1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y+1));
	floorObstacle->polygon.vertices.push_back(Geometry2d::Point(x, y));
	_nonFloor[1] = ObstaclePtr(floorObstacle);

	y = Constants::Floor::Length;
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
	const float halfFlat = Constants::Field::GoalFlat/2.0;
	const float radius = Constants::Field::ArcRadius;
	goalArea->polygon.vertices.push_back(Geometry2d::Point(-halfFlat, 0));
	goalArea->polygon.vertices.push_back(Geometry2d::Point(-halfFlat, radius));
	goalArea->polygon.vertices.push_back(Geometry2d::Point( halfFlat, radius));
	goalArea->polygon.vertices.push_back(Geometry2d::Point( halfFlat, 0));
	_goalArea[0] = ObstaclePtr(goalArea);
	_goalArea[1] = ObstaclePtr(new CircleObstacle(Geometry2d::Point(-halfFlat, 0), radius));
	_goalArea[2] = ObstaclePtr(new CircleObstacle(Geometry2d::Point(halfFlat, 0), radius));

	// Create robots
	for (size_t i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		self[i] = new Robot(this, i, true);
		opp[i] = new Robot(this, i, false);
	}
}

Gameplay::GameplayModule::~GameplayModule()
{
	removeGoalie();
	
	for (size_t i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		delete self[i];
		delete opp[i];
	}
}

void Gameplay::GameplayModule::createGoalie()
{
	if (!_goalie)
	{
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

void Gameplay::GameplayModule::run()
{
	QMutexLocker lock(&_mutex);
	
	bool verbose = false;
	if (verbose) cout << "Starting GameplayModule::run()" << endl;

	// perform state variable updates on robots
	BOOST_FOREACH(Robot* robot, self)
	{
		if (robot) {
			robot->update();
		}
	}

	ObstaclePtr largeBallObstacle;
	ObstaclePtr smallBallObstacle;
	if (_state->ball.valid)
	{
		largeBallObstacle = ObstaclePtr(new CircleObstacle(_state->ball.pos, Constants::Field::CenterRadius));
		smallBallObstacle = ObstaclePtr(new CircleObstacle(_state->ball.pos, Constants::Ball::Radius));
	}

	ObstaclePtr selfObstacles[Constants::Robots_Per_Team];
	ObstaclePtr oppObstacles[Constants::Robots_Per_Team];
	for (size_t i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		//FIXME - These should not be in Gameplay.  This should be the responsibility of motion planning.
		if (_state->self[i].valid)
		{
			selfObstacles[i] = ObstaclePtr(new CircleObstacle(_state->self[i].pos, Constants::Robot::Radius - .01));
		}

		if (_state->opp[i].valid)
		{
			oppObstacles[i] = ObstaclePtr((new CircleObstacle(_state->opp[i].pos, Constants::Robot::Radius - .01)));
		}
	}

	// Assign the goalie
	bool newGoalie = false;
	if (_goalie && !_goalie->assigned())
	{
		set<Robot *> robots;
		BOOST_FOREACH(Robot *r, self)
		{
			robots.insert(r);
		}
		
		// The Goalie behavior is responsible for only selecting a robot which is allowed by the rules
		// (no changing goalies at random times).
		// The goalie behavior has priority for choosing robots because it must obey this rule,
		// so the current play will be restarted in case the goalie stole one of its robots.
		if (_goalie->assign(robots))
		{
			newGoalie = true;
		}
	}

	// Set up robots before the play is run
	BOOST_FOREACH(Robot *r, self)
	{
		//robot resets
		r->willKick = false;
		r->avoidBall = false;

		// Reset the motion command
		r->resetMotionCommand();

		// Add obstacles for this robot
		ObstacleGroup &obstacles = r->packet()->obstacles;
		obstacles.clear();

		if (r->visible())
		{
			// Add rule-based obstacles (except for the ball, which will be added after the play
			// has a change to set willKick and avoidBall)
			for (size_t i = 0; i < Constants::Robots_Per_Team; ++i)
			{
				if (self[i] != r && selfObstacles[i])
				{
					obstacles.add(selfObstacles[i]);
				}

				if (!r->approachOpponent[i] && oppObstacles[i])
				{
					obstacles.add(oppObstacles[i]);
				}
			}

			//if not a goalie, avoid our goalie area
			if (!_goalie || _goalie->robot() != r)
			{
				BOOST_FOREACH(ObstaclePtr& ptr, _goalArea)
				{
					obstacles.add(ptr);
				}
			}

			if (_state->gameState.stayOnSide())
			{
				obstacles.add(_sideObstacle);
			}

			// Add non floor obstacles
			BOOST_FOREACH(ObstaclePtr& ptr, _nonFloor)
			{
				obstacles.add(ptr);
			}
		}
	}

	_ballMatrix = Geometry2d::TransformMatrix::translate(_state->ball.pos);

	if (verbose) cout << "  Updating play" << endl;
	
	// important scenarios:
	//  - no current plays - must pick a new one
	//  - no available plays - must do nothing
	//  - current play is fine, must be run
	//  - current play has ended, must select new and run
	//  - current play is not applicable/failed, must kill, select new and run
	//  - new goalie, select new play (may reselect current play, which much be assigned new robots)

	// prepare a list of all non-goalie robots ready
	_robots.clear();
	int n = 0;
	BOOST_FOREACH(Robot *r, self)
	{
		if ((!_goalie || r != _goalie->robot()) && r->visible() && !r->exclude)
		{
			_robots.insert(r);
			++n;
		}
		
		if (n == Constants::Robots_Per_Team - 1)
		{
			// Leave one available to become the goalie, in case the goalie broke
			break;
		}
	}
	if (verbose) cout << "  Available Robots: " << _robots.size() << endl;

	// Update play scores.
	// The GUI thread needs this for the Plays tab and we will use it later
	// to determine the new play.
	BOOST_FOREACH(PlayFactory *factory, PlayFactory::factories())
	{
		factory->lastScore = factory->score(this);
	}

	if (
			newGoalie ||							// Goalie gets first pick of all robots
			_playDone || 							// New play if old one was complete
			!_currentPlay ||						// There is no current play
			isinf(_currentPlayFactory->lastScore) || // Current play doesn't apply anymore
			!_currentPlay->allVisible() ||			// Lost robots used by current play
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
		// Restart the current play if the goalie changed, because we have to pick new robots.
		if (bestPlay)
		{
			if (bestPlay != _currentPlayFactory || newGoalie)
			{
				if (verbose) cout << "  Assigning robots to play" << endl;

				_currentPlayFactory = bestPlay;
				_currentPlay = shared_ptr<Play>(_currentPlayFactory->create(this));
			}
		} else {
			// No usable plays
			_currentPlay.reset();
			_currentPlayFactory = 0;
		}
	}

	// Run the current play
	if (_currentPlay)
	{
		if (verbose) cout << "  Running play" << endl;
		_playDone = !_currentPlay->run();
	}

	// Run the goalie
	if (_goalie)
	{
		if (verbose) cout << "  Running goalie" << endl;
		if (_goalie->assigned() && _goalie->robot()->visible())
		{
			_goalie->run();
		}
	}

	// Add ball obstacles
	if (verbose) cout << "  Adding ball obstacles" << endl;
	BOOST_FOREACH(Robot *r, self)
	{
		if (r->visible() && !(_goalie && _goalie->robot() == r))
		{
			// Any robot that isn't the goalie may have to avoid the ball
			if ((_state->gameState.state != GameState::Playing && !_state->gameState.ourRestart) || r->avoidBall)
			{
				// Opponent's restart: always stay away from the ball
				if (largeBallObstacle)
				{
					r->obstacles().add(largeBallObstacle);
					r->obstacles().add(smallBallObstacle);
				}
			} else if (!r->willKick)
			{
				// Don't hit the ball unintentionally during normal play
				if (smallBallObstacle)
				{
					r->obstacles().add(smallBallObstacle);
				}
			}
		}
	}

	if (_state->gameState.stayAwayFromBall() && _state->ball.valid)
	{
		_state->drawCircle(_state->ball.pos, Constants::Field::CenterRadius, Qt::black, "Rules");
	}

	if (_currentPlay)
	{
		_playName = _currentPlay->name();
	} else {
		_playName = QString();
	}
	if (verbose) cout << "Finishing GameplayModule::run()" << endl;
}
