// kate: indent-mode cstyle; indent-width 4; tab-width 4; space-indent false;
// vim:ai ts=4 et

#include <iostream>
#include <limits>
#include <boost/foreach.hpp>
#include "RobotModel.hpp"
#include <Utils.hpp>

using namespace std;

//#define KALMAN

Modeling::RobotModel::RobotModel(const ConfigFile::WorldModel& cfg, int s) :
//	posA(6,6), posB(6,6), posP(6,6), posQ(6,6), posR(2,2), posH(2,6),
//	posZ(2), posU(6), posE(6), posX0(6), angA(3,3), angB(3,3), angP(3,3),
//	angQ(3,3), angR(1,1), angH(1,3), angZ(1), angU(3), angE(3), angX0(3),
	_shell(s), _angle(0), _angleVel(0), _angleAccel(0), _config(cfg), _haveBall(false)

{
#ifdef KALMAN

	/** Position **/
	posA.zero();
	posB.zero();
	posQ.zero();
	posR.zero();
	posH.zero();
	posP.zero();
	posX0.zero();
	posU.zero();

	//Alpha-Beta-Gamma Filter
	posAlpha = _config.pos.alpha;
	posBeta = _config.pos.beta;
	posGamma = _config.pos.gamma;

	//Process covariance between position and velocity (E[x,x_dot])
// 	posQ(1,0) = posQ(4,3) = 10;

	//Process covariance between velocity and acceleration (E[x)dot,x_ddot])
// 	posQ(2,1) = posQ(5,4) = 0.01;

	//Process covariance between position and acceleration (E[x,x_ddot])
// 	posQ(2,0) = posQ(5,3) = 0.001;

	//Process covariance (E[x,x], E[x_dot,x_dot], etc)
	posQ(0,0) = posQ(3,3) = 10;

	posQ(1,1) = posQ(4,4) = 10;

	posQ(2,2) = posQ(5,5) = 10;

	//Measurement Covariance for position in the x and the y
	posR(0,0) = posR(1,1) = 0.1;

	//Measurement Model. We can only measure position
	posH(0,0) = posH(1,3) = 1;

	//Initial Covariance indicates we have no idea about our state;
	//X
	posP(0,0) = posP(0,1) = posP(0,2) = posP(1,0) = posP(1,1) = posP(1,2) = posP(2,0) = posP(2,1) = posP(2,2) = 500;
	//Y
	posP(3,3) = posP(3,4) = posP(3,5) = posP(4,3) = posP(4,4) = posP(4,5) = posP(5,3) = posP(5,4) = posP(5,5) = 500;

	//State Model
	posA(0,0) = posA(1,1) = posA(2,2) = posA(3,3) = posA(4,4) = posA(5,5) = 1;

	posKalman = new DifferenceKalmanFilter(&posA, &posB, &posX0, &posP, &posQ, &posR, &posH);

	/** Angle **/
	angA.zero();
	angB.zero();
	angQ.zero();
	angR.zero();
	angH.zero();
	angP.zero();
	angX0.zero();
	angU.zero();

	//Angle ABG Stuff
	angleAlpha = _config.angle.alpha;
	angleBeta = _config.angle.beta;
	angleGamma = _config.angle.gamma;

	//Process covariance between position and velocity (E[x,x_dot])
// 	angQ(1,0) = 10;

	//Process covariance between velocity and acceleration (E[x)dot,x_ddot])
// 	angQ(2,1) = 1;

	//Process covariance between position and acceleration (E[x,x_ddot])
// 	angQ(2,0) = 0.1;

	//Process covariance (E[x,x], E[x_dot,x_dot], etc)
	angQ(0,0) = 10;

	angQ(1,1) = 10;

	angQ(2,2) = 10;

	//Measurement Covariance for position in the x and the y
	angR(0,0) = 0.1;

	//Measurement Model. We can only measure position
	angH(0,0) = 1;

	//Initial Covariance indicates we have no idea about our state;
	angP(0,0) = angP(0,1) = angP(0,2) = angP(1,0) = angP(1,1) = angP(1,2) = angP(2,0) = angP(2,1) = angP(2,2) = 500;

	//State Model
	angA(0,0) = angA(1,1) = angA(2,2) = 1;

	angKalman = new DifferenceKalmanFilter(&angA, &angB, &angX0, &angP, &angQ, &angR, &angH);
# else
	//Alpha-Beta-Gamma Filter
	_posAlpha = _config.pos.alpha;
	_posBeta = _config.pos.beta;
	_posGamma = _config.pos.gamma;

	_angleAlpha = _config.angle.alpha;
	_angleBeta = _config.angle.beta;
	_angleGamma = _config.angle.gamma;
#endif
	lastObservedTime = 0;

}

bool Modeling::RobotModel::valid(uint64_t cur_time) const {
	return !_observations.empty() || (cur_time - lastObservedTime) > MaxRobotCoastTime;
}

void Modeling::RobotModel::observation(uint64_t time, Geometry2d::Point pos, float angle)
{
	Observation_t obs = {pos, angle, time};
	_observations.push_back(obs);
}

Geometry2d::Point Modeling::RobotModel::predictPosAtTime(float dtime)
{
	return _pos + _vel * dtime + _accel * 0.5f * dtime * dtime;
}

void Modeling::RobotModel::update(uint64_t cur_time)
{
	// create time differential between this frame and the last
	float dtime = (float)(cur_time - lastObservedTime) / 1e6;
	lastObservedTime = cur_time;

	// prediction step from previous frame
	Geometry2d::Point predictPos = predictPosAtTime(dtime);
	Geometry2d::Point predictVel = _vel + _accel * dtime;

	float predictAngle = _angle + _angleVel * dtime + _angleAccel * 0.5f * dtime * dtime;
	float predictAngleVel = _angleVel + _angleAccel * dtime;

	// if we have no observations, coast and return
	if (_observations.empty()) {
		_pos = predictPos;
		_vel = predictVel;
		_angle = predictAngle;
		_angleVel = predictAngleVel;
		return;
	}

	// sort the observations to find the lowest error (cheap method)
	// Alternative: average them
	Geometry2d::Point observedPos = _observations.front().pos;
	float observedAngle = _observations.front().angle;
	float bestError = std::numeric_limits<float>::infinity();
	BOOST_FOREACH(const Observation_t& obs, _observations) {
		float error = (obs.pos - predictPos).magsq()
				+ fabs(Utils::fixAngleDegrees(obs.angle - predictAngle));
		if (error < bestError) {
			observedPos = obs.pos;
			observedAngle = obs.angle;
			bestError = error;
		}
	}

	// Perform filter updates
	if (dtime)
	{
	#ifndef KALMAN

		// determine error using observations
		Geometry2d::Point posError = observedPos - predictPos;
		_pos = predictPos + posError * _posAlpha;
		_vel = predictVel + posError * _posBeta / dtime;
		_accel += posError * _posGamma / (dtime * dtime);

		float angleError = Utils::fixAngleDegrees(observedAngle - predictAngle);
		_angle = Utils::fixAngleDegrees(predictAngle + angleError * _angleAlpha);
		_angleVel = predictAngleVel + angleError * _angleBeta / dtime;
		_angleAccel += angleError * _angleGamma / (dtime * dtime);

	#else
		/** Position **/
		//Update model
		posA(0,1) = dtime;
		posA(0,2) = 0.5*dtime*dtime;

		posA(1,2) = dtime;

		posA(3,4) = dtime;
		posA(3,5) = 0.5*dtime*dtime;

		posA(4,5) = dtime;

		//Position
		posZ(0) = observedPos.x;
		posZ(1) = observedPos.y;
		RobotModel::shared &robot = p.second;

		posKalman->predict(&posU);
		posKalman->correct(&posZ);

		_pos.x = (float)posKalman->state()->elt(0);
		_vel.x = (float)posKalman->state()->elt(1);
		_accel.x = (float)posKalman->state()->elt(2);
		_pos.y = (float)posKalman->state()->elt(3);
		_vel.y = (float)posKalman->state()->elt(4);
		_accel.y = (float)posKalman->state()->elt(5);

		//Using the ABG filter for independent velocity and acceleration
// 		Geometry2d::Point predictPos = abgPos + vel * dtime + accel * 0.5f * dtime * dtime;
//         Geometry2d::Point predictVel = vel + accel * dtime;
//
// 		Geometry2d::Point posError = observedPos - predictPos;
//         abgPos = predictPos + posError * posAlpha;
//         vel = predictVel + posError * posBeta / dtime;
//         accel += posError * posGamma / (dtime * dtime);

		/** Angle **/
		//Update model
		angA(0,1) = dtime;
		angA(0,2) = 0.5*dtime*dtime;

		angA(1,2) = dtime;

		angB(0,1) = angB(1,2) = dtime;
		angB(1,1) = 1;

		//Position
		angZ(0) = observedAngle;

		angKalman->predict(&angU);
		angKalman->correct(&angZ);

		_angle = (float)angKalman->state()->elt(0);
		_angleVel = (float)angKalman->state()->elt(1);
		_angleAccel = (float)angKalman->state()->elt(2);

	#endif
	}

	// cleanup by removing old observations
	_observations.clear();
}
