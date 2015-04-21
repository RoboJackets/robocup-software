
#include <iostream>

#include <LinearAlgebra.hpp>

#include "rbpf/Rbpf.hpp"
#include "rbpf/RbpfModelRolling.hpp"
#include "rbpf/RbpfModelKicked.hpp"

#include "RBPFBallModel.hpp"

using namespace std;
using namespace Geometry2d;
using namespace rbpf;

Modeling::RBPFBallModel::RBPFBallModel(RobotModel::RobotMap *robotMap, Configuration *config) :
		BallModel(robotMap, config)
{
	// Construct initial state X (n x 1)
	VectorNd X = VectorNd::Zero();
	// Construct initial state covariance P (n x n)
	MatrixNNd P; P.setIdentity(); P *= 0.01;
	// Create Rbpf
	int numParticles = 10; // Number of particles in filter
	raoBlackwellizedParticleFilter = new Rbpf(X,P,numParticles);
	// create model graph
	raoBlackwellizedParticleFilter->addModel(new RbpfModelRolling(_robotMap, config));
	raoBlackwellizedParticleFilter->addModel(new RbpfModelKicked(_robotMap, config));
	raoBlackwellizedParticleFilter->setTransProb(0,0,0.9);
	raoBlackwellizedParticleFilter->setTransProb(0,1,0.1);
	raoBlackwellizedParticleFilter->setTransProb(1,0,0.9);
	raoBlackwellizedParticleFilter->setTransProb(1,1,0.1);
}

Modeling::RBPFBallModel::~RBPFBallModel()
{
	if (raoBlackwellizedParticleFilter)
	{
		delete raoBlackwellizedParticleFilter;
	}
}

void Modeling::RBPFBallModel::singleUpdate(float dtime) {
	raoBlackwellizedParticleFilter->update(observedPos.x,observedPos.y,dtime);
	RbpfState* bestState = raoBlackwellizedParticleFilter->getBestFilterState();
	pos.x = bestState->X(0);
	pos.y = bestState->X(1);
	vel.x = bestState->X(2);
	vel.y = bestState->X(3);
	accel.x = bestState->X(4);
	accel.y = bestState->X(5);
}

void Modeling::RBPFBallModel::update(float dtime) {
	if(_observations.size() >= 1){ // currently hacked to just handle a single update
		// pick the closest observation to the current estimate
		float bestDist = 99999;
		for (const observation_type& observation :  _observations)
		{
			if(observation.pos.distTo(pos) < bestDist)
			{
				bestDist = observation.pos.distTo(pos);
				observedPos = observation.pos;
			}
		}
		float dtime = (float)(_observations.at(0).time - lastUpdatedTime) / 1e6;
		singleUpdate(dtime);
	}
	/*
	if(obs.size() <= 0){
		return;
	}

	cout << "received " << obs.size() << " observations" << std::endl;

	int numObs = obs.size();
	double x[numObs];
	double y[numObs];
	double dt[numObs];
	for(int i=obs.size()-1; i>=0; i--){
		x[i] = obs[i].pos.x;
		y[i] = obs[i].pos.y;
		dt[i] = (float)(obs[i].time - lastUpdatedTime) / 1e6;
	}
	raoBlackwellizedParticleFilter->updateMultipleObs(x,y,dt,numObs);
	RbpfState* bestState = raoBlackwellizedParticleFilter->getBestFilterState();
	pos.x = bestState->X(0);
	pos.y = bestState->X(1);
	vel.x = bestState->X(2);
	vel.y = bestState->X(3);
	accel.x = bestState->X(4);
	accel.y = bestState->X(5);*/
}
