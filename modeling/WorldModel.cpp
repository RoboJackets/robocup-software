// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
#include "WorldModel.hpp"

#include <QObject>
#include <QString>
#include <vector>

#include "RobotWM.hpp"
#include "framework/Module.hpp"
//#include "Ball.hpp"

using namespace Modeling;

WorldModel::WorldModel(ConfigFile::RobotFilterCfg cfg) :
  Module("World Model")
{
    _cfg = cfg;

    for(unsigned int i=0 ; i<5 ; i++)
    {
      _self[i] = new RobotWM(_cfg, true);
      _opp[i] = new RobotWM(_cfg, false);
    }

}


void WorldModel::run()
{
	Q_FOREACH(const Packet::Vision& vision, _state->rawVision)
	{
		if (!vision.sync)
		{
			const std::vector<Packet::Vision::Robot>* self = &vision.blue;
			const std::vector<Packet::Vision::Robot>* opp = &vision.yellow;
			
			if (_state->team == Yellow)
			{
				self = &vision.yellow;
				opp = &vision.blue;
			} 
			else if (_state->team == UnknownTeam)
			{
				//TODO need to not act..or something
			}
			
			//index is the id
			Q_FOREACH (const Packet::Vision::Robot& r, *self)
			{
				//FIXME - getting shell 255
				if (r.shell > 5)
				{   
					continue;
				}

				//get data from vision packet
				_self[r.shell]->_shell = r.shell;
				_self[r.shell]->_measPos = r.pos;
				_self[r.shell]->_measAngle = r.angle;
				_self[r.shell]->_valid = true;

				//get data from commands
				_self[r.shell]->_cmdVel = _state->self[r.shell].cmd.vel;
				_self[r.shell]->_cmdAngle = _state->self[r.shell].cmd.angle;
			  
				//process world model
				_self[r.shell]->process();

				//store data back in state variable
				_state->self[r.shell].shell = _self[r.shell]->_shell;
				_state->self[r.shell].pos = _self[r.shell]->_pos;
				_state->self[r.shell].vel = _self[r.shell]->_vel;
				_state->self[r.shell].angle = _self[r.shell]->_posAngle;
				_state->self[r.shell].angleVel = _self[r.shell]->_velAngle;
				_state->self[r.shell].valid = true;
			}
			//index is the id
			Q_FOREACH (const Packet::Vision::Robot& r, *opp)
			{
				//FIXME - getting shell 255
				if (r.shell > 5)
				{   
					continue;
				}

				//get data from vision packet
				_opp[r.shell]->_shell = r.shell;
				_opp[r.shell]->_measPos = r.pos;
				_opp[r.shell]->_measAngle = r.angle;
				_opp[r.shell]->_valid = true;
				//Note that control information cannot be used for opp

				//process world model
				_opp[r.shell]->process();

				//store data back in state variable
				_state->opp[r.shell].shell = _opp[r.shell]->_shell;
				_state->opp[r.shell].pos = _opp[r.shell]->_pos;
				_state->opp[r.shell].vel = _opp[r.shell]->_vel;
				_state->opp[r.shell].angle = _opp[r.shell]->_posAngle;
				_state->opp[r.shell].angleVel = _opp[r.shell]->_velAngle;
				_state->opp[r.shell].valid = true;
			}
			
			//copy ball
			if (vision.balls.size() > 0)
			{
				_state->ball.pos = vision.balls[0].pos;
				_state->ball.valid = true;
			}
		}
	}
}
