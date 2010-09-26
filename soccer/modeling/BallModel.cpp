// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include "BallModel.hpp"

#include <Configuration.hpp>

#include <iostream>

#include <boost/foreach.hpp>

using namespace std;

Modeling::BallModel::BallModel(RobotModel::RobotMap *robotMap, Configuration *config) :
		_robotMap(robotMap)
{
	lastObservedTime = 0;
	lastUpdatedTime = 0;
}

Modeling::BallModel::~BallModel()
{
}

Geometry2d::Point Modeling::BallModel::predictPosAtTime(float dtime)
{
	//return pos + vel * dtime + accel * 0.5f * dtime * dtime;
	// because the approximation of accel is poor, just predict based on velocity and position
	return pos+vel*dtime;
}

void Modeling::BallModel::observation(uint64_t time, const Geometry2d::Point &pos, observation_mode obs_type)
{
	if(time < lastUpdatedTime){
		// due to an issue with early measurements coming in late, this will
		// discard any observations that happened before the previous update
		return;
	}
	
	observation_type obs = {time, pos, obs_type};
	_observations.push_back(obs);

	// set lastObservedTime to the last observation's time
	if (time >= lastObservedTime)
		lastObservedTime = time;
}

bool Modeling::BallModel::valid(uint64_t time) {
	//cout << "ball model is " << (!_observations.empty() || ((time - lastUpdatedTime) < MaxCoastTime) ? "" : "not") << " valid" << std::endl;
	return !_observations.empty() && ((time - lastUpdatedTime) < MaxCoastTime);
}

void Modeling::BallModel::run(uint64_t time)
{
	const bool verbose = false;

	float dtime = 0.0;
	if(_observations.empty()) {
		// coast the ball using simple integrator
		dtime = (float)(time - lastUpdatedTime) / 1e6;
		pos = predictPosAtTime(dtime);
		vel += accel * 0.5f * dtime * dtime;
		return;
	}

	// loop through the observations and determine if we got vision observations
	bool gotCameraObservation = false;
	BOOST_FOREACH(const observation_type& obs, _observations) {
		if(obs.obs_type == BallModel::VISION){
			gotCameraObservation = true;
			break;
		}
	}

	// If there are camera observations, remove robot sensor observations.
	vector<observation_type> goodObs;
	if(gotCameraObservation){
		BOOST_FOREACH(const observation_type& obs, _observations) {
			if (obs.obs_type == BallModel::VISION) {
				goodObs.push_back(obs);
			}
		}
	} else {
		goodObs = _observations;
	}
	_observations = goodObs;
	if (verbose) cout << " ballModel has " << _observations.size() << " observations, updating..." << endl;

	/** combine the measurements using a subclass */
	this->update(dtime);

	// remove previous observations after processing them
	_observations.clear();

	// remember last update
	lastUpdatedTime = time;
}
