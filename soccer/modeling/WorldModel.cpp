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

//#include "Ball.hpp"

using namespace std;
using namespace Modeling;
using namespace Utils;
using namespace google::protobuf;

// Maximum time to coast a track (keep the track alive with no observations) in microseconds.
const uint64_t MaxCoastTime = 500000;

WorldModel::WorldModel(SystemState *state, ConfigFile::shared_worldmodel cfg) :
	_state(state),
	_selfPlayers(Constants::Robots_Per_Team), _oppPlayers(Constants::Robots_Per_Team),
	ballModel(BallModel::RBPF, &_robotMap, cfg),
	_config(cfg)
{
}

WorldModel::~WorldModel()
{
}

void WorldModel::run(bool blueTeam, const std::vector<const SSL_DetectionFrame *> &rawVision)
{
	// internal verbosity flag for debugging
	bool verbose = false;

	if (verbose) cout << "In WorldModel::run()" << endl;

	// Add vision packets
	uint64_t curTime = 0;
	if (verbose) cout << "Adding vision packets" << endl;
	BOOST_FOREACH(const SSL_DetectionFrame* vision, rawVision)
	{
		//FIXME - This time is not usable here.  It is in the vision computer's clock.
		uint64_t timestamp = vision->t_capture() * 1000000.0;
		curTime = max(curTime, timestamp);
		
		// determine team
		const RepeatedPtrField<SSL_DetectionRobot> *self, *opp;
		if (blueTeam)
		{
			self = &vision->robots_blue();
			opp = &vision->robots_yellow();
		} else {
			self = &vision->robots_yellow();
			opp = &vision->robots_blue();
		}

		// add ball observation
		BOOST_FOREACH(const SSL_DetectionBall &ball, vision->balls())
		{
			Geometry2d::Point pos(ball.x() / 1000.0f, ball.y() / 1000.0f);
			ballModel.observation(timestamp, pos, BallModel::VISION);
		}

		// add robot observations
		BOOST_FOREACH(const SSL_DetectionRobot &robot, *self)
		{
			addRobotObseration(robot, timestamp, _selfPlayers);
		}
		BOOST_FOREACH(const SSL_DetectionRobot &robot, *opp)
		{
			addRobotObseration(robot, timestamp, _oppPlayers);
		}
	}

	// get robot data from return packets
	if (verbose) cout << "Adding robot rx data" << endl;
	BOOST_FOREACH(SystemState::Robot& robot, _state->self) {
		addRobotRxData(robot);
	}

	// Robots perform updates
	if (verbose) cout << "updating players" << endl;
	updateRobots(_selfPlayers, curTime);
	updateRobots(_oppPlayers, curTime);

	// Copy robot data out of models into state
	if (verbose) cout << "copying out data for robots" << endl;
	copyRobotState(_selfPlayers, SELF);
	copyRobotState(_oppPlayers, OPP);

	// Store the robot models for use by the ball model
	_robotMap.clear();
	BOOST_FOREACH(const RobotModel::shared& model, _selfPlayers)
	{
		if (model)
		{
			_robotMap[model->shell()] = model;
		}
	}
	
	BOOST_FOREACH(const RobotModel::shared& model, _oppPlayers)
	{
		if (model)
		{
			_robotMap[model->shell() + OppOffset] = model;
		}
	}

	// add observations to the ball based on ball sensors and filtered robot positions
	if (verbose) cout << "adding ball observations for ball sensors" << endl;
	BOOST_FOREACH(RobotModel::shared robot, _selfPlayers)
	{
		if (robot) {
			//if a robot has the ball, we need to make an observation
#if 0
			//FIXME - They're breaking.  Turn this back on later.
			if (robot->valid(curTime) && robot->hasBall())
			{
				Geometry2d::Point offset = Geometry2d::Point::
						direction(robot->angle() * DegreesToRadians) *	Constants::Robot::Radius;

				ballModel.observation(_state->timestamp, robot->pos() + offset, BallModel::BALL_SENSOR);
			}
#endif
		}
	}

	if (verbose) cout << "Updating ball" << endl;
	bool ballValid = ballModel.valid(curTime);
	ballModel.update(curTime);

	_state->ball.pos = ballModel.pos;
	_state->ball.vel = ballModel.vel;
	_state->ball.accel = ballModel.accel;
	_state->ball.valid = ballValid;

	if (verbose) cout << "At end of WorldModel::run()" << endl;
}

void WorldModel::addRobotObseration(const SSL_DetectionRobot &obs, uint64_t timestamp, RobotVector& players)
{
	int obs_shell = obs.robot_id();

	// try to add to an existing model, and return if we update something
	BOOST_FOREACH(RobotModel::shared& model, players) {
		if (model && model->shell() == obs_shell) {
			Geometry2d::Point pos(obs.x() / 1000.0f, obs.y() / 1000.0f);
			model->observation(timestamp, pos, obs.orientation());
			return;
		}
	}

	// find an open slot - assumed that invalid models will have been removed already
	BOOST_FOREACH(RobotModel::shared& model, players) {
		if (!model) {
			model = RobotModel::shared(new RobotModel(_config, obs_shell));
			Geometry2d::Point pos(obs.x() / 1000.0f, obs.y() / 1000.0f);
			model->observation(timestamp, pos, obs.orientation());
			return;
		}
	}
}

void WorldModel::updateRobots(vector<RobotModel::shared>& players, uint64_t cur_time) {
	BOOST_FOREACH(RobotModel::shared& model, players) {
		if (model && model->valid(cur_time)) {
			model->update(cur_time);
		} else {
			model = RobotModel::shared();
		}
	}
}

void WorldModel::addRobotRxData(SystemState::Robot& log_robot) {
	int shell = log_robot.shell;
	BOOST_FOREACH(RobotModel::shared& model, _selfPlayers) {
		if (model && model->shell() == shell) {
			model->hasBall(log_robot.radioRx.ball());
			return;
		}
	}
}

void WorldModel::copyRobotState(const std::vector<RobotModel::shared>& players, TeamMode m) {
	unsigned int i=0;
	SystemState::Robot* log = (m == SELF) ? _state->self : _state->opp;
	BOOST_FOREACH(const RobotModel::shared& robot, players) {
		if (robot) {
			log[i].valid = true;
			log[i].shell = robot->shell();
			log[i].pos = robot->pos();
			log[i].vel = robot->vel();
			log[i].angle = robot->angle();
			log[i].angleVel = robot->angleVel();
		} else {
			log[i].valid = false;
		}
		++i;
	}
}
