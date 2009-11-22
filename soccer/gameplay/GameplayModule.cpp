// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

// Behaviors in use:
// 	forward
// 	fullback
// 	idle
// 	intercept
// 	kick
// 	kickoff
// 	move
// 	penalty

#include "PlayConfigTab.hpp"
#include "GameplayModule.hpp"
#include "Behavior.hpp"
#include "behaviors/positions/Goalie.hpp"

// TO INSERT PLAYS: Add the include here, grouped by test plays and real plays
// real plays
#include "plays/OurKickoff.hpp"
#include "plays/TheirKickoff.hpp"
#include "plays/OurFreekick.hpp"
#include "plays/TheirFreekick.hpp"
#include "plays/DefendPenalty.hpp"
#include "plays/KickPenalty.hpp"
#include "plays/Stopped.hpp"
#include "plays/Offense.hpp"
#include "plays/Defense.hpp"

// test plays
#include "plays/test_plays/TestBasicPassing.hpp"
#include "plays/test_plays/TestBasicAttack.hpp"

#include <QMouseEvent>
#include <QFileDialog>

#include <assert.h>
#include <cmath>
#include <Constants.hpp>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using namespace std;
using namespace boost;
using namespace Utils;

Gameplay::GameplayModule::GameplayModule(SystemState *state):
	Module("Gameplay")
{
	_state = state;
	_goalie = 0;
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
	for (int i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		self[i] = new Robot(this, i, true);
		opp[i] = new Robot(this, i, false);
	}

	// Create play configuration tab
	_playConfig = new PlayConfigTab();
	_playConfig->gameplay = this;
	_widget = _playConfig;
	
	// Create the goalie - on by default
	createGoalie();

	// TO INSERT PLAYS: add the play as below

	// Create a set of available normal plays
	_playConfig->addPlay(make_shared<Plays::OurKickoff>(this));
	_playConfig->addPlay(make_shared<Plays::TheirKickoff>(this));
	_playConfig->addPlay(make_shared<Plays::OurFreekick>(this));
	_playConfig->addPlay(make_shared<Plays::TheirFreekick>(this));
	_playConfig->addPlay(make_shared<Plays::KickPenalty>(this));
	_playConfig->addPlay(make_shared<Plays::DefendPenalty>(this));
	_playConfig->addPlay(make_shared<Plays::Stopped>(this));
	_playConfig->addPlay(make_shared<Plays::Offense>(this));
	_playConfig->addPlay(make_shared<Plays::Defense>(this));

	// Add testing plays
	_playConfig->addPlay(make_shared<Plays::TestBasicPassing>(this));
	_playConfig->addPlay(make_shared<Plays::TestBasicAttack>(this));
}

Gameplay::GameplayModule::~GameplayModule()
{
	removeGoalie();
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

void Gameplay::GameplayModule::fieldOverlay(QPainter &painter, Packet::LogFrame &frame) const
{
	// Referee rules
	painter.setPen(Qt::black);
	if (frame.gameState.stayAwayFromBall() && frame.ball.valid)
	{
		painter.drawEllipse(frame.ball.pos.toQPointF(), Constants::Field::CenterRadius, Constants::Field::CenterRadius);
	}
}

void Gameplay::GameplayModule::run()
{
	_state->debugLines.clear();
	_state->debugPolygons.clear();
	
	ObstaclePtr largeBallObstacle;
	ObstaclePtr smallBallObstacle;
	if (_state->ball.valid)
	{
		largeBallObstacle = ObstaclePtr(new CircleObstacle(_state->ball.pos, Constants::Field::CenterRadius));
		smallBallObstacle = ObstaclePtr(new CircleObstacle(_state->ball.pos, Constants::Ball::Radius));
	}

	ObstaclePtr selfObstacles[Constants::Robots_Per_Team];
	ObstaclePtr oppObstacles[Constants::Robots_Per_Team];
	for (int i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		//FIXME - Use polygons to extend in time
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
	if (_goalie && !_goalie->assigned())
	{
		//FIXME - The rules allow for changing the goalie only in certain circumstances.  Make sure we do this right.
		BOOST_FOREACH(Robot *r, self)
		{
			bool inUse = false;
			if (_currentPlay)
			{
				const std::set<Robot *> &playRobots = _currentPlay->robots();
				if (playRobots.find(r) != playRobots.end())
				{
					inUse = true;
				}
			}
			
			if (r->visible() && !inUse)
			{
				printf("Goalie is robot %d\n", r->id());
				_goalie->assignOne(r);
				break;
			}
		}
	}
	
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
			for (int i = 0; i < Constants::Robots_Per_Team; ++i)
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

	// Select a play
	if (_playDone || !_currentPlay || !_currentPlay->applicable())
	{
		_playDone = false;
		
		shared_ptr<Play> play = selectPlay();
		if (play != _currentPlay)
		{
			_currentPlay = play;
			if (_currentPlay)
			{
// 				updateCurrentPlay(QString::fromStdString(_currentPlay->name()));
				
				// Assign to the new play all robots except the goalie
				set<Robot *> robots;
				BOOST_FOREACH(Robot *r, self)
				{
					if (!_goalie || r != _goalie->robot())
					{
						robots.insert(r);
					}
				}
				
				_currentPlay->assign(robots);
			}
		}
	}
	
	// Run the current play
	if (_currentPlay)
	{
		_playDone = !_currentPlay->run();
	}
	
	// Run the goalie
	if (_goalie)
	{
		if (_goalie->assigned() && _goalie->robot()->visible())
		{
			_goalie->run();
		}
	}

	// Add ball obstacles
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
	
	if (_currentPlay)
	{
		_state->playName = _currentPlay->name();
	} else {
		_state->playName = "(null)";
	}
}

void Gameplay::GameplayModule::enablePlay(shared_ptr<Play> play)
{
	QMutexLocker lock(&_playMutex);
	_plays.insert(play);
}

void Gameplay::GameplayModule::disablePlay(shared_ptr<Play> play)
{
	QMutexLocker lock(&_playMutex);
	_plays.erase(play);
}

bool Gameplay::GameplayModule::playEnabled(boost::shared_ptr<Play> play) const
{
	QMutexLocker lock(&_playMutex);
	return _plays.find(play) != _plays.end();
}

shared_ptr<Gameplay::Play> Gameplay::GameplayModule::selectPlay()
{
	float bestScore = 0;
	shared_ptr<Play> bestPlay;
	
	// Find the best applicable play
	BOOST_FOREACH(shared_ptr<Play> play, _plays)
	{
		if (play->applicable())
		{
			float score = play->score();
			if (!bestPlay || score < bestScore)
			{
				bestScore = score;
				bestPlay = play;
			}
		}
	}
	
	return bestPlay;
}
