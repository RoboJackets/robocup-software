// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <iostream>
#include <boost/foreach.hpp>

#include "Rbpf.hpp"
#include "RbpfModelRolling.hpp"
#include "RbpfModelKicked.hpp"

#include "RBPFBallModel.hpp"

/* RBPF Includes */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

using namespace std;
using namespace Geometry2d;

Modeling::RBPFBallModel::RBPFBallModel(RobotModel::RobotMap *robotMap, Configuration *config) :
		BallModel(robotMap, config)
{
	typedef boost::numeric::ublas::vector<double> Vector;
	typedef boost::numeric::ublas::matrix<double> Matrix;

	// Construct initial state X (n x 1)
	Vector X(6); X.clear();
	// Construct initial state covariance P (n x n)
	Matrix P(6,6); P.clear(); P(0,0)=P(1,1)= P(2,2)=P(3,3)=P(4,4)=P(5,5)=0.01;
	// Create Rbpf
	int numParticles = 20; // Number of particles in filter
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
	Point posOld = pos;
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
		BOOST_FOREACH(const observation_type& observation, _observations)
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
