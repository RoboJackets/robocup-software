// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#include "WorldModel.hpp"

#include <QObject>
#include <QString>
#include <vector>
#include <boost/foreach.hpp>

#include <Constants.hpp>
#include <Utils.hpp>

#include "framework/Module.hpp"

//#include "Ball.hpp"

using namespace std;
using namespace Modeling;
using namespace Utils;

// Maximum time to coast a track (keep the track alive with no observations) in microseconds.
const uint64_t MaxCoastTime = 500000;

WorldModel::WorldModel(SystemState *state, ConfigFile::WorldModel& cfg) :
	Module("World Model"),
	_config(cfg)
{
	_state = state;
	for(unsigned int i=0 ; i<5 ; i++)
	{
		_selfRobot[i] = 0;
		_oppRobot[i] = 0;
	}
}

WorldModel::~WorldModel()
{
	for(unsigned int i=0 ; i<5 ; i++)
	{
		//delete _self[i];
		//delete _opp[i];
	}
	
	BOOST_FOREACH(RobotMap::value_type p, _robotMap)
	{
		RobotModel *robot = p.second;
		delete robot;
	}
}

void WorldModel::run()
{
#if 1
	// Reset errors on all tracks
	BOOST_FOREACH(RobotMap::value_type p, _robotMap)
	{
		RobotModel *robot = p.second;
		robot->bestError = -1;
	}
	ballModel.bestError = -1;
	
	/// ball sensor
	
	BOOST_FOREACH(Packet::LogFrame::Robot& r, _state->self)
	{
		r.haveBall = r.radioRx.ball;
		
		//if a robot has the ball, we need to make an observation 
		//using that information and project the ball in front of it
		if (r.valid && r.haveBall)
		{
			Geometry2d::Point offset = Geometry2d::Point::
				direction(r.angle * DegreesToRadians) *	Constants::Robot::Radius;
			
			ballModel.observation(_state->timestamp, r.pos + offset);
		}
	}
	
	//printf("--- WorldModel\n");
	uint64_t curTime = 0;
	BOOST_FOREACH(const Packet::Vision& vision, _state->rawVision)
	{
		curTime = max(curTime, vision.timestamp);
		
		if (!vision.sync)
		{
			const std::vector<Packet::Vision::Robot>* self;
			const std::vector<Packet::Vision::Robot>* opp;
			
			if (_state->team == Yellow)
			{
				self = &vision.yellow;
				opp = &vision.blue;
			} else if (_state->team == Blue)
			{
				self = &vision.blue;
				opp = &vision.yellow;
			} else {
				continue;
			}
			
			BOOST_FOREACH(const Packet::Vision::Robot& r, *self)
			{
				RobotModel *robot = map_lookup(_robotMap, r.shell);
				if (!robot)
				{
					robot = new RobotModel(_config, r.shell);
					_robotMap[r.shell] = robot;
				}
				robot->observation(vision.timestamp, r.pos, r.angle);
			}
			
			BOOST_FOREACH(const Packet::Vision::Robot& r, *opp)
			{
				RobotModel *robot = map_lookup(_robotMap, r.shell + OppOffset);
				if (!robot)
				{
					robot = new RobotModel(_config, r.shell);
					_robotMap[r.shell + OppOffset] = robot;
				}
				robot->observation(vision.timestamp, r.pos, r.angle);
			}
			
			BOOST_FOREACH(const Packet::Vision::Ball &ball, vision.balls)
			{
				ballModel.observation(vision.timestamp, ball.pos);
			}
		}
	}
	
	// Update/delete robots
	vector<RobotModel *> selfUnused;
	vector<RobotModel *> oppUnused;
	for (RobotMap::iterator i = _robotMap.begin(); i != _robotMap.end();)
	{
		RobotMap::iterator next = i;
		++next;
		
		RobotModel *robot = i->second;
		if ((curTime - robot->lastObservedTime) > MaxCoastTime)
		{
			// This robot is too old, so delete it.
			if (robot->report)
			{
				*robot->report = 0;
			}
			
			delete robot;
			_robotMap.erase(i);
		} else {
			if (robot->bestError >= 0)
			{
				// This robot has a new observation.  Update it.
				robot->update();
				
				if (!robot->report)
				{
					if (i->first < OppOffset)
					{
						selfUnused.push_back(robot);
					} else {
						oppUnused.push_back(robot);
					}
				}
			}
		}
		
		i = next;
	}
	
	//FIXME - Sort unused lists
	
	unsigned int nextSelfUnused = 0;
	unsigned int nextOppUnused = 0;
	for (int i = 0; i < 5; ++i)
	{
		if (!_selfRobot[i] && nextSelfUnused < selfUnused.size())
		{
			_selfRobot[i] = selfUnused[nextSelfUnused++];
			_selfRobot[i]->report = &_selfRobot[i];
		}
		
		if (_selfRobot[i])
		{
			_state->self[i].valid = true;
			_state->self[i].shell = _selfRobot[i]->shell;
			_state->self[i].pos = _selfRobot[i]->pos;
			_state->self[i].vel = _selfRobot[i]->vel;
			_state->self[i].angle = _selfRobot[i]->angle;
			_state->self[i].angleVel = _selfRobot[i]->angleVel;
		} else {
			_state->self[i].valid = false;
		}
		
		if (!_oppRobot[i] && nextOppUnused < oppUnused.size())
		{
			_oppRobot[i] = oppUnused[nextOppUnused++];
			_oppRobot[i]->report = &_oppRobot[i];
		}
		
		if (_oppRobot[i])
		{
			_state->opp[i].valid = true;
			_state->opp[i].shell = _oppRobot[i]->shell;
			_state->opp[i].pos = _oppRobot[i]->pos;
			_state->opp[i].vel = _oppRobot[i]->vel;
			_state->opp[i].angle = _oppRobot[i]->angle;
			_state->opp[i].angleVel = _oppRobot[i]->angleVel;
		} else {
			_state->opp[i].valid = false;
		}
	}
	
	ballModel.update();
	
	_state->ball.pos = ballModel.pos;
	_state->ball.vel = ballModel.vel;
	_state->ball.accel = ballModel.accel;
	_state->ball.valid = (curTime - ballModel.lastObservedTime) < MaxCoastTime;
	
#else
	BOOST_FOREACH(const Packet::Vision& vision, _state->rawVision)
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
			BOOST_FOREACH (const Packet::Vision::Robot& r, *self)
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
			BOOST_FOREACH (const Packet::Vision::Robot& r, *opp)
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
#endif
}
