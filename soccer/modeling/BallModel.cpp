// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <stdio.h>

#include "BallModel.hpp"

#define KALMAN 1


Modeling::BallModel::BallModel() :
	A(6,6), B(6,6), P(6,6), Q(6,6), R(2,2), H(2,6),
	Z(2), U(6), X0(6)
{
	bestError = 0;
	bestObservedTime = 0;
	lastObservedTime = 0;
	missedFrames = 0;

#if KALMAN
	A.zero();
	B.zero();
	Q.zero();
	R.zero();
	H.zero();
	P.zero();
	X0.zero();

	alpha = 1;
	beta = .4;
	gamma = .1;

	//Process covariance between position and velocity (E[x,x_dot])
// 	Q(1,0) = Q(4,3) = 0.01;

	//Process covariance between velocity and acceleration (E[x)dot,x_ddot])
// 	Q(2,1) = Q(5,4) = 0.001;

	//Process covariance between position and acceleration (E[x,x_ddot])
// 	Q(2,0) = Q(5,3) = 0.0001;

	//Process covariance (E[x,x], E[x_dot,x_dot], etc)
	Q(0,0) = Q(3,3) = 10;

	Q(1,1) = Q(4,4) = 10;

	Q(2,2) = Q(5,5) = 10;

	//Measurement Covariance for position in the x and the y
	R(0,0) = R(1,1) = 0.1;

	//Measurement Model. We can only measure position
	H(0,0) = H(1,3) = 1;

	//State Model
	A(0,0) = A(1,1) = A(2,2) = A(3,3) = A(4,4) = A(5,5) = 1;

	posKalman = new DifferenceKalmanFilter(&A, &B, &X0, &P, &Q, &R, &H);
#else
	/*
	 * Initialize Rao-Blackwellized Particle Filter
	 *   Constructs initial state X, initial covariance P, adds several models
	 *   to the modelGraph, and sets some transition weights
	 */
	// Construct initial state X (n x 1)
	Vector X(6); X*=0;
	// Construct initial state covariance P (n x n)
	Matrix P(6,6); P*=0.0; P(0,0)=P(1,1)= P(2,2)=P(3,3)=P(4,4)=P(5,5)=0.1;
	// Create Rbpf
	int numParticles = 20; // Number of particles in filter
	raoBlackwellizedParticleFilter = new Rbpf(X,P,numParticles);
	// create model graph
	raoBlackwellizedParticleFilter->addModel(new RbpfModelRollingFriction());
	raoBlackwellizedParticleFilter->addModel(new RbpfModelKicked());
	raoBlackwellizedParticleFilter->setTransProb(0,0,0.9);
	raoBlackwellizedParticleFilter->setTransProb(0,1,0.1);
	raoBlackwellizedParticleFilter->setTransProb(1,0,0.9);
	raoBlackwellizedParticleFilter->setTransProb(1,1,0.1);
#endif
}

Geometry2d::Point Modeling::BallModel::predictPosAtTime(float dtime)
{
	return pos + vel * dtime + accel * 0.5f * dtime * dtime;
}

void Modeling::BallModel::observation(uint64_t time, const Geometry2d::Point &pos)
{
	if (lastObservedTime)
	{
		float dt = (float)(time - lastObservedTime) / 1e6;
		float error = (pos - predictPosAtTime(dt)).magsq();
//		float error = (pos - this->pos).magsq();
		if (bestError < 0 || error < bestError)
		{
			bestError = error;
			observedPos = pos;
			bestObservedTime = time;
		}
	} else {
		// First observation
		bestError = 0;
		this->pos = pos;
		observedPos = pos;
		bestObservedTime = time;
		lastObservedTime = time;
	}
}

void Modeling::BallModel::update()
{
	float dtime = (float)(bestObservedTime - lastObservedTime) / 1e6;

	if (missedFrames > 5 || bestError < (0.2 * 0.2))
	{
		lastObservedTime = bestObservedTime;

		if (dtime)
		{
		#if KALMAN
			//Update model
			A(0,1) = dtime;
			A(0,2) = 0.5*dtime*dtime;

			A(1,2) = dtime;

			A(3,4) = dtime;
			A(3,5) = 0.5*dtime*dtime;

			A(4,5) = dtime;

			//Position
			Z(0) = observedPos.x;
			Z(1) = observedPos.y;

			posKalman->predict(&U);
			posKalman->correct(&Z);

			pos.x = (float)posKalman->state()->elt(0);
			vel.x = (float)posKalman->state()->elt(1);
			accel.x = (float)posKalman->state()->elt(2);
			pos.y = (float)posKalman->state()->elt(3);
			vel.y = (float)posKalman->state()->elt(4);
			accel.y = (float)posKalman->state()->elt(5);

// 			Geometry2d::Point predictPos = abgPos + vel * dtime + accel * 0.5f * dtime * dtime;
// 			Geometry2d::Point predictVel = vel + accel * dtime;

// 			Geometry2d::Point posError = observedPos - predictPos;
// 			abgPos = predictPos + posError * alpha;
// 			vel = predictVel + posError * beta / dtime;
// 			accel += posError * gamma / (dtime * dtime);
		#else
			raoBlackwellizedParticleFilter->update(observedPos.x,observedPos.y,dtime);
			RbpfState* bestState = raoBlackwellizedParticleFilter->getBestFilterState();
			pos.x = bestState->X(0);
			pos.y = bestState->X(1);
			vel.x = bestState->X(2);
			vel.y = bestState->X(3);
			accel.x = bestState->X(4);
			accel.y = bestState->X(5);
		#endif

		}
	} else {
		// Ball moved too far to possibly be a valid track, so just extrapolate from the last known state
		if (dtime)
		{
			pos += vel * dtime + accel * 0.5f * dtime * dtime;
		}

		++missedFrames;
	}
	bestError = -1;
}
