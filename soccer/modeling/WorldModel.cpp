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

WorldModel::WorldModel(SystemState *state, const ConfigFile::WorldModel& cfg) :
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
		//FIXME: handle stale data properly
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
}
