// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et
#include "WorldModel.hpp"

#include <iostream>
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
	ballModel(BallModel::RBPF, &_robotMap),
	_config(cfg)
{
	_state = state;
	_selfSlots.assign(5, -1);
	_oppSlots.assign(5, -1);
}

WorldModel::~WorldModel()
{
}

void WorldModel::run()
{
	// internal verbosity flag for debugging
	bool verbose = false;

	if (verbose) cout << "In WorldModel::run()" << endl;
	// Reset errors on all tracks
	BOOST_FOREACH(RobotMap::value_type p, _robotMap)
	{
		RobotModel::shared robot = p.second;
		robot->bestError = -1;
	}
	ballModel.bestError = -1;

	/// ball sensor
	// FIXME: need to check for consistency here - could be a broken sensor
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

	if (verbose) cout << "Adding messages from vision " << endl;
	uint64_t curTime = 0;
	BOOST_FOREACH(const Packet::Vision& vision, _state->rawVision)
	{
		curTime = max(curTime, vision.timestamp);

		if (!vision.sync)
		{
			const std::vector<Packet::Vision::Robot> * self, * opp;

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
				RobotModel::shared &robot = _robotMap[r.shell];
				if (!robot.get()) {
					cout << "creating self robot " << (int) r.shell << endl;
					robot = RobotModel::shared(new RobotModel(_config, r.shell));
				}
				robot->observation(vision.timestamp, r.pos, r.angle);
			}

			BOOST_FOREACH(const Packet::Vision::Robot& r, *opp)
			{
				RobotModel::shared &robot = _robotMap[r.shell + OppOffset];
				if (!robot.get()) {
					cout << "creating opp robot " << (int) r.shell << endl;
					robot = RobotModel::shared(new RobotModel(_config, r.shell));
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
	if (verbose) cout << "Sorting robots" << endl;
	vector<RobotModel::shared> selfUnused, oppUnused;
	BOOST_FOREACH(RobotMap::value_type& p, _robotMap)
	{
		int shell = p.first;
		RobotModel::shared &robot = p.second;
		if ((curTime - robot->lastObservedTime) < MaxCoastTime) // && robot->bestError >= 0)
		{
			// This robot has a new observation.  Update it.
			robot->update();
			robot->isValid = true;

			// check if previously unused robot
			if (!robot->inUse)
			{
				if (shell < OppOffset)
				{
					selfUnused.push_back(robot);
				} else {
					oppUnused.push_back(robot);
				}
			}
		} else {
			if (robot->isValid) cout << "Robot " << robot->shell << " out of date, removing..." << endl;
			// robot is out of date, set flag
			robot->inUse = false;
			robot->isValid = false;
		}
	}

	if (verbose) cout << "Assigning robots to slots" << endl;
	unsigned int nextSelfUnused = 0, nextOppUnused = 0;
	for (int i = 0; i < 5; ++i)
	{
		// SELF ROBOTS

		// clear out invalid robots
		if (_selfSlots[i] >= 0 && _robotMap[_selfSlots[i]] && !_robotMap[_selfSlots[i]]->isValid)
		{
			_selfSlots[i] = -1;
		}

		// fill in slots
		if (_selfSlots[i] < 0 && nextSelfUnused < selfUnused.size()) {
			RobotModel::shared robot = selfUnused[nextSelfUnused++];
			_selfSlots[i] = robot->shell;
			robot->inUse = true;
		}

		// copy in the robot data
		if (_selfSlots[i] >= 0) {
			RobotModel::shared robot = _robotMap[_selfSlots[i]];
			_state->self[i].valid = true;
			_state->self[i].shell = robot->shell;
			_state->self[i].pos = robot->pos;
			_state->self[i].vel = robot->vel;
			_state->self[i].angle = robot->angle;
			_state->self[i].angleVel = robot->angleVel;
		}

		// OPPONENT ROBOTS

		// clear out invalid robots
		if (_oppSlots[i] >= 0 && _robotMap[_oppSlots[i] + OppOffset] &&
				!_robotMap[_oppSlots[i] + OppOffset]->isValid)
		{
			_oppSlots[i] = -1;
		}

		// fill in slots
		if (_oppSlots[i] < 0 && nextOppUnused < oppUnused.size()) {
			RobotModel::shared robot = oppUnused[nextOppUnused++];
			_oppSlots[i] = robot->shell;
			robot->inUse = true;
		}

		// copy in the robot data
		if (_oppSlots[i] >= 0) {
			RobotModel::shared robot = _robotMap[_oppSlots[i] + OppOffset];
			_state->opp[i].valid = true;
			_state->opp[i].shell = robot->shell;
			_state->opp[i].pos = robot->pos;
			_state->opp[i].vel = robot->vel;
			_state->opp[i].angle = robot->angle;
			_state->opp[i].angleVel = robot->angleVel;
		}

	}
	if (verbose) cout << "Updating ball" << endl;

	ballModel.update();

	_state->ball.pos = ballModel.pos;
	_state->ball.vel = ballModel.vel;
	_state->ball.accel = ballModel.accel;
	_state->ball.valid = (curTime - ballModel.lastObservedTime) < MaxCoastTime;

	if (verbose) cout << "At end of WorldModel::run()" << endl;
}
