#include "WorldModel.hpp"

#include <QObject>
#include <iostream>
#include <vector>

#include "Robot.hpp"
#include "Ball.hpp"

using namespace Modeling;

WorldModel::WorldModel() :
	Module("World Model")
{

//   blueTeam = new std::vector<Robot*>;
//   yellowTeam = new std::vector<Robot*>;
//   for (unsigned int i = 0; i<5; ++i) 
//     {
//     blueTeam->push_back(new Robot());
//     yellowTeam->push_back(new Robot());
//     }
//   ball = new Ball();

}

void WorldModel::run()
{

  Q_FOREACH(const Packet::Vision& vision, _state->rawVision)
    {
      	if (!vision.sync)
	{
	  if (_state->isBlue)
	    {
	      //index is the id
	      Q_FOREACH (const Packet::Vision::Robot& r, vision.blue)
		{
		  _state->self[r.shell].shell = r.shell;
		  _state->self[r.shell].pos = r.pos;
		  _state->self[r.shell].angle = r.angle;
		  _state->self[r.shell].valid = true;
		}
	      //index is the id
	      Q_FOREACH (const Packet::Vision::Robot& r, vision.yellow)
		{
		  _state->opp[r.shell].shell = r.shell;
		  _state->opp[r.shell].pos = r.pos;
		  _state->opp[r.shell].angle = r.angle;
		  _state->opp[r.shell].valid = true;
		}
	    }
	  else
	    {
	      //index is the id
	      Q_FOREACH (const Packet::Vision::Robot& r, vision.blue)
		{
		  _state->opp[r.shell].shell = r.shell;
		  _state->opp[r.shell].pos = r.pos;
		  _state->opp[r.shell].angle = r.angle;
		  _state->opp[r.shell].valid = true;
		}
	      //index is the id
	      Q_FOREACH (const Packet::Vision::Robot& r, vision.yellow)
		{
		  _state->self[r.shell].shell = r.shell;
		  _state->self[r.shell].pos = r.pos;
		  _state->self[r.shell].angle = r.angle;
		  _state->self[r.shell].valid = true;
		}
	    }
	  //copy ball
	  _state->ball.pos = vision.balls.at(1).pos;
	}
    }
}
