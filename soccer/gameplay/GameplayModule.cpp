// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "GameplayModule.hpp"
#include "Predicates.hpp"
#include "Named_Matrix.hpp"
#include "behaviors/positions/Goalie.hpp"

#include <QMouseEvent>

#include <assert.h>
#include <cmath>
#include <Constants.hpp>

#include <boost/foreach.hpp>

using namespace std;
using namespace Utils;

// Centered on the ball
Gameplay::Named_Matrix ball_matrix("ball");

// Center of the field
Gameplay::Named_Matrix center_matrix("center");

// Opponent's coordinates
Gameplay::Named_Matrix opp_matrix("opp");

Gameplay::GameplayModule::GameplayModule():
	Module("Gameplay"),
	playbook(this)
{
	_availableRobots = 0;

	Predicates::always = true;

	center_matrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Constants::Field::Length / 2));
	opp_matrix = Geometry2d::TransformMatrix::translate(Geometry2d::Point(0, Constants::Field::Length)) *
				Geometry2d::TransformMatrix::rotate(180);

	for (int r = 0; r < Constants::Robots_Per_Team; ++r)
	{
		self[r] = new Robot(this, r, true);
		opp[r] = new Robot(this, r, false);
	}

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

	//TESTING
	Predicates::offense = true;
}

void Gameplay::GameplayModule::createGoalie()
{
	if (!playbook.goalie())
	{
		playbook.goalie(new Behaviors::Goalie(this, 0));
	}
}

void Gameplay::GameplayModule::loadPlays(const char *dir)
{
	playbook.loadDir(dir);
}

void Gameplay::GameplayModule::loadPlay(const char *path)
{
	playbook.load(path);
}

void Gameplay::GameplayModule::fieldOverlay(QPainter &painter, Packet::LogFrame &frame) const
{
    // Referee rules
    painter.setPen(Qt::black);
    if (_state->gameState.stayAwayFromBall() && _state->ball.valid)
    {
        painter.drawEllipse(_state->ball.pos.toQPointF(), Constants::Field::CenterRadius, Constants::Field::CenterRadius);
    }
}

void Gameplay::GameplayModule::run()
{
	// Put behavior names in log frame
	for (int i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		Behavior *b = self[i]->behavior();
		if (b)
		{
			_state->self[i].behavior = b->name();
		} else {
			_state->self[i].behavior.clear();
		}
	}
	
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

	for (int r = 0; r < Constants::Robots_Per_Team; ++r)
	{
		//robot resets
		self[r]->willKick = false;
		self[r]->avoidBall = false;
		self[r]->state()->cmd.vScale = 1.0;

		// Make each robot stand still.
		// Manual controlled robots do not need this to happen.
		if (_selectedRobotId == -1 || _selectedRobotId != r)
		{
			_state->self[r].cmd.goalPosition = _state->self[r].pos;
			_state->self[r].cmd.pivot = Packet::LogFrame::MotionCmd::NoPivot;
		}

		// Add obstacles for this robot
		_state->self[r].obstacles.clear();

		if (_state->self[r].valid)
		{
			// Add rule-based obstacles (except for the ball, which will be added after the play
			// has a change to set willKick and avoidBall)
			for (int i = 0; i < Constants::Robots_Per_Team; ++i)
			{
				if (i != r && selfObstacles[i])
				{
					_state->self[r].obstacles.add(selfObstacles[i]);
				}

				if (!self[r]->approachOpponent[i] && oppObstacles[i])
				{
					_state->self[r].obstacles.add(oppObstacles[i]);
				}
			}

			//if not a goalie, avoid our goalie area
			if (!(playbook.goalie() && playbook.goalie()->robot() &&
				r == playbook.goalie()->robot()->id()))
			{
				BOOST_FOREACH(ObstaclePtr& ptr, _goalArea)
				{
					_state->self[r].obstacles.add(ptr);
				}
			}

			if (_state->gameState.stayOnSide())
			{
				_state->self[r].obstacles.add(_sideObstacle);
			}

			// Add non floor obstacles
			BOOST_FOREACH(ObstaclePtr& ptr, _nonFloor)
			{
				_state->self[r].obstacles.add(ptr);
			}
		}
	}

	// Count how many robots are available.
	_availableRobots = 0;
	for (int i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		if (self[i]->visible())
		{
			_availableRobots++;
		}
	}

	// Set predicates
	Predicates::have2 = (_availableRobots >= 2);
	Predicates::have3 = (_availableRobots >= 3);
	Predicates::have4 = (_availableRobots >= 4);

	Predicates::stopped = (_state->gameState.state == GameState::Stop);
	Predicates::setup = (_state->gameState.state == GameState::Setup);
	Predicates::ready = (_state->gameState.state == GameState::Ready);
	Predicates::restart = _state->gameState.setupRestart();
	Predicates::playing = (_state->gameState.state == GameState::Playing);

	bool restart = _state->gameState.setupRestart();
	Predicates::kickoff = restart && _state->gameState.kickoff();
	Predicates::penalty = restart && _state->gameState.penalty();
	Predicates::direct = restart && _state->gameState.direct();
	Predicates::indirect = restart && _state->gameState.indirect();
	Predicates::freekick = restart && (_state->gameState.direct() || _state->gameState.indirect());
	Predicates::our_restart = restart && _state->gameState.ourRestart;

	Predicates::winning = _state->gameState.ourScore > _state->gameState.theirScore;
	Predicates::losing = _state->gameState.ourScore < _state->gameState.theirScore;

	if (restart && _state->gameState.ourRestart)
	{
		Predicates::offense = true;
	} else if (restart && !_state->gameState.ourRestart)
	{
		Predicates::offense = false;
	}

	// thresholds
	float angle_thresh = 20 * DegreesToRadians;
	float dist_thresh = Constants::Robot::Radius + Constants::Ball::Radius + 0.1;
	float speed_thresh = 1.0;

	Geometry2d::Point ball_pos = _state->ball.pos;
	Geometry2d::Point ball_vel = _state->ball.vel;

	//Determine if we have the ball to set offense
	//Only change state if we have a team has changed possession
	if (Predicates::offense)
	{ //check if the other team now has the ball
		BOOST_FOREACH(Robot *r, opp)
		{
			//opp has ball if it is close to a robot and moving in the same direction
			float dist = ball_pos.distTo(r->pos());
			float vel_angle_diff = abs(fixAngleRadians(ball_vel.angle() - r->vel().angle()));
			bool ball_downfield = ball_pos.y < r->pos().y - Constants::Robot::Radius;
			bool isSameSpeed = abs(r->vel().mag() - ball_vel.mag()) < speed_thresh;

			if (dist < dist_thresh &&
					vel_angle_diff < angle_thresh &&
					ball_downfield &&
					isSameSpeed)
			{
				Predicates::offense = false;
				break;
			}
		}
	}
	else
	{
		//check if we have acquired the ball, then set offense predicate
		BOOST_FOREACH(Robot * r, self)
		{
			float dist = ball_pos.distTo(r->pos());
			float vel_angle_diff = abs(fixAngleRadians(ball_vel.angle() - r->vel().angle()));
			bool ball_downfield = ball_pos.y > (r->pos().y + Constants::Robot::Radius);
			
			if (r->state()->haveBall ||
					(dist < dist_thresh &&
							vel_angle_diff < angle_thresh &&
							ball_downfield))
			{
				Predicates::offense = true;
				break;
			}
		}
	}

	//Determine if there is a free ball
	float free_ball_thresh = 0.5;
	float free_ball_vel_thresh = 1.0;
	Predicates::free_ball = true;
	BOOST_FOREACH(Robot *r, opp)
	{
		if (ball_pos.nearPoint(r->pos(), free_ball_thresh) &&
				ball_vel.mag() < free_ball_vel_thresh)
		{
			Predicates::free_ball = false;
			break;
		}
	}
	
	if (!Predicates::free_ball)
	{
		BOOST_FOREACH(Robot *r, self)
		{
			if (ball_pos.nearPoint(r->pos(), free_ball_thresh) &&
					ball_vel.mag() < free_ball_vel_thresh)
			{
				Predicates::free_ball = true;
				break;
			}
		}
	}

	//Set field position predicates
	if (ball_pos.y < Constants::Field::Length /3)
	{
		Predicates::home_field = true;
		Predicates::mid_field = false;
		Predicates::opp_field = false;
	}
	else if (ball_pos.y < Constants::Field::Length * 2/3 &&
			ball_pos.y > Constants::Field::Length /3)
	{
		Predicates::home_field = false;
		Predicates::mid_field = true;
		Predicates::opp_field = false;
	}
	else
	{
		Predicates::home_field = false;
		Predicates::mid_field = false;
		Predicates::opp_field = true;
	}

	ball_matrix = Geometry2d::TransformMatrix::translate(_state->ball.pos);

	// Select a new play if necessary
	if (playbook.setup())
	{
		// Selected a new play
	}

	// Run the current play
	playbook.run();

	// Add ball obstacles
	for (int r = 0; r < Constants::Robots_Per_Team; ++r)
	{
		if (_state->self[r].valid)
		{
			Behavior *goalie = playbook.goalie();
			if (!goalie || goalie->robot() != self[r])
			{
				// Any robot that isn't the goalie may have to avoid the ball
				if ((_state->gameState.state != GameState::Playing && !_state->gameState.ourRestart) || self[r]->avoidBall)
				{
					// Opponent's restart: always stay away from the ball
					if (largeBallObstacle)
					{
						_state->self[r].obstacles.add(largeBallObstacle);
						_state->self[r].obstacles.add(smallBallObstacle);
					}
				} else if (!self[r]->willKick)
				{
					// Don't hit the ball unintentionally during normal play
					if (smallBallObstacle)
					{
						_state->self[r].obstacles.add(smallBallObstacle);
					}
				}
			}
		}
	}
}

void Gameplay::GameplayModule::mousePress(QMouseEvent* me, Geometry2d::Point pos)
{
    if (me->button() == Qt::LeftButton && _selectedRobotId != -1)
    {
        //_state->self[_selectedRobotId].cmd.goalPosition = pos;
    }
}

void Gameplay::GameplayModule::mouseMove(QMouseEvent* me, Geometry2d::Point pos)
{
}

void Gameplay::GameplayModule::mouseRelease(QMouseEvent* me, Geometry2d::Point pos)
{
}

Gameplay::Robot *Gameplay::GameplayModule::find(const std::string &name)
{
	for (int i = 0; i < 5; ++i)
	{
		if (self[i]->name() == name)
		{
			return self[i];
		}
	}

	for (int i = 0; i < 5; ++i)
	{
		if (opp[i]->name() == name)
		{
			return opp[i];
		}
	}

	return 0;
}
