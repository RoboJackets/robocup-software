#include "WorldModel.hpp"

#include <QObject>
#include <vector>

//#include "Robot.hpp"
//#include "Ball.hpp"

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
			const std::vector<Packet::Vision::Robot>* self = &vision.blue;
			const std::vector<Packet::Vision::Robot>* opp = &vision.yellow;
			
			if (!_state->isBlue)
			{
				self = &vision.yellow;
				opp = &vision.blue;
			}
			
			//index is the id
			Q_FOREACH (const Packet::Vision::Robot& r, *self)
			{
				_state->self[r.shell].shell = r.shell;
				_state->self[r.shell].pos = r.pos;
				_state->self[r.shell].angle = r.angle;
				_state->self[r.shell].valid = true;
			}
			//index is the id
			Q_FOREACH (const Packet::Vision::Robot& r, *opp)
			{
				_state->opp[r.shell].shell = r.shell;
				_state->opp[r.shell].pos = r.pos;
				_state->opp[r.shell].angle = r.angle;
				_state->opp[r.shell].valid = true;
			}
			
			//copy ball
			if (vision.balls.size() > 0)
			{
				_state->ball.pos = vision.balls.at(1).pos;
			}
		}
	}
}
