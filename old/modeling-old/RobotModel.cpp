#include <iostream>
#include <limits>
#include "RobotModel.hpp"
#include <Utils.hpp>

using namespace std;

Modeling::RobotModel::Config::Config(Configuration* config)
    : posAlpha(config->createDouble("RobotModel/Position/Alpha", 1)),
      posBeta(config->createDouble("RobotModel/Position/Beta", 0)),
      posGamma(config->createDouble("RobotModel/Position/Gamma", 0)),
      angleAlpha(config->createDouble("RobotModel/Angle/Alpha", 1)),
      angleBeta(config->createDouble("RobotModel/Angle/Beta", 0)),
      angleGamma(config->createDouble("RobotModel/Angle/Gamma", 0)) {}

Modeling::RobotModel::RobotModel(Config* config, int s)
    :
#ifdef KALMANMODEL
      posA(6, 6),
      posB(6, 6),
      posP(6, 6),
      posQ(6, 6),
      posR(2, 2),
      posH(2, 6),
      posZ(2),
      posU(6),
      posE(6),
      posX0(6),
      angA(3, 3),
      angB(3, 3),
      angP(3, 3),
      angQ(3, 3),
      angR(1, 1),
      angH(1, 3),
      angZ(1),
      angU(3),
      angE(3),
      angX0(3),
#endif
      _shell(s),
      _angle(0),
      _angleVel(0),
      _angleAccel(0),
      _config(config),
      _haveBall(false) {
#ifdef KALMANMODEL

    /** Position **/
    posA.zero();
    posB.zero();
    posQ.zero();
    posR.zero();
    posH.zero();
    posP.zero();
    posX0.zero();
    posU.zero();

    // Process covariance between position and velocity (E[x,x_dot])
    posQ(1, 0) = posQ(4, 3) = _config->covPosVel;

    // Process covariance between velocity and acceleration (E[x)dot,x_ddot])
    posQ(2, 1) = posQ(5, 4) = _config->covVelAcc;

    // Process covariance between position and acceleration (E[x,x_ddot])
    posQ(2, 0) = posQ(5, 3) = _config->covPosAcc;

    // Process covariance (E[x,x], E[x_dot,x_dot], etc)
    posQ(0, 0) = posQ(3, 3) = _config->covPos;

    posQ(1, 1) = posQ(4, 4) = _config->covVel;

    posQ(2, 2) = posQ(5, 5) = _config->covAcc;

    // Measurement Covariance for position in the x and the y
    posR(0, 0) = posR(1, 1) = _config->measurementNoise;

    // Measurement Model. We can only measure position
    posH(0, 0) = posH(1, 3) = 1;

    // Initial Covariance indicates we have no idea about our state;
    // X
    posP(0, 0) = posP(0, 1) = posP(0, 2) = posP(1, 0) = posP(1, 1) =
        posP(1, 2) = posP(2, 0) = posP(2, 1) = posP(2, 2) = 500;
    // Y
    posP(3, 3) = posP(3, 4) = posP(3, 5) = posP(4, 3) = posP(4, 4) =
        posP(4, 5) = posP(5, 3) = posP(5, 4) = posP(5, 5) = 500;

    // State Model
    posA(0, 0) = posA(1, 1) = posA(2, 2) = posA(3, 3) = posA(4, 4) =
        posA(5, 5) = 1;

    _posKalman = new DifferenceKalmanFilter(&posA, &posB, &posX0, &posP, &posQ,
                                            &posR, &posH);

    /** Angle **/
    angA.zero();
    angB.zero();
    angQ.zero();
    angR.zero();
    angH.zero();
    angP.zero();
    angX0.zero();
    angU.zero();

    // Process covariance between position and velocity (E[x,x_dot])
    // 	angQ(1,0) = 10;

    // Process covariance between velocity and acceleration (E[x)dot,x_ddot])
    // 	angQ(2,1) = 1;

    // Process covariance between position and acceleration (E[x,x_ddot])
    // 	angQ(2,0) = 0.1;

    // Process covariance (E[x,x], E[x_dot,x_dot], etc)
    angQ(0, 0) = 10;

    angQ(1, 1) = 10;

    angQ(2, 2) = 10;

    // Measurement Covariance for position in the x and the y
    angR(0, 0) = 0.1;

    // Measurement Model. We can only measure position
    angH(0, 0) = 1;

    // Initial Covariance indicates we have no idea about our state;
    angP(0, 0) = angP(0, 1) = angP(0, 2) = angP(1, 0) = angP(1, 1) =
        angP(1, 2) = angP(2, 0) = angP(2, 1) = angP(2, 2) = 500;

    // State Model
    angA(0, 0) = angA(1, 1) = angA(2, 2) = 1;

    _angKalman = new DifferenceKalmanFilter(&angA, &angB, &angX0, &angP, &angQ,
                                            &angR, &angH);
#endif
    lastObservedTime = 0;
    lastUpdatedTime = 0;
}

bool Modeling::RobotModel::valid(uint64_t cur_time) const {
    return !_observations.empty() ||
           (cur_time - lastObservedTime) < MaxRobotCoastTime;
}

void Modeling::RobotModel::observation(uint64_t time, Geometry2d::Point pos,
                                       float angle) {
    // if(rand() % 2 != 1){cout << "skipping" << std::endl; return;} // testing
    // - simulate bad obs

    // ignore obserations at exactly (0,0)
    if (!(pos.x == 0.0f && pos.y == 0.0f)) {
        Observation_t obs = {pos, angle, time};
        _observations.push_back(obs);
    }

    // set lastObservedTime to the last observation's time
    if (time > lastObservedTime) lastObservedTime = time;
}

Geometry2d::Point Modeling::RobotModel::predictPosAtTime(float dtime) {
    return _pos + _vel * dtime + _accel * 0.5f * dtime * dtime;
}

void Modeling::RobotModel::update(uint64_t cur_time) {
    bool verbose = false;
    // debug by printing observations
    if (verbose) {
        cout << "Updating RobotModel with observations: ";
        for (const Observation_t obs : _observations) {
            cout << "(" << obs.pos.x << ", " << obs.pos.y << ") ";
        }
        cout << endl;
    }

    // create time differential between this frame and the last
    float dtime = (float)(cur_time - lastUpdatedTime) / 1e6;

    // prediction step from previous frame
    Geometry2d::Point predictPos = predictPosAtTime(dtime);
    Geometry2d::Point predictVel = _vel + _accel * dtime;

    float predictAngle =
        _angle + _angleVel * dtime + _angleAccel * 0.5f * dtime * dtime;
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
    // Alternative: average themangKalman = new DifferenceKalmanFilter(&angA,
    // &angB, &angX0, &angP, &angQ, &angR, &angH);
    Geometry2d::Point observedPos = _observations.front().pos;
    float observedAngle = _observations.front().angle;
    float bestError = std::numeric_limits<float>::infinity();
    for (const Observation_t& obs : _observations) {
        float error = (obs.pos - predictPos).magsq() +
                      fabs(fixAngleDegrees(obs.angle - predictAngle));
        if (error < bestError) {
            observedPos = obs.pos;
            observedAngle = obs.angle;
            bestError = error;
        }
    }

    if (verbose)
        cout << "   Updating Filter (" << dtime << ") - best filter obs: ("
             << observedPos.x << ", " << observedPos.y << ")" << endl;

    // Perform filter updates
    if (dtime) {
#ifndef KALMANMODEL

        // determine error using observations
        Geometry2d::Point posError = observedPos - predictPos;
        _pos = predictPos + posError * *_config->posAlpha;
        _vel = predictVel + posError * *_config->posBeta / dtime;
        _accel += posError * *_config->posGamma / (dtime * dtime);

        float angleError = fixAngleDegrees(observedAngle - predictAngle);
        _angle =
            fixAngleDegrees(predictAngle + angleError * *_config->angleAlpha);
        _angleVel = predictAngleVel + angleError * *_config->angleBeta / dtime;
        _angleAccel += angleError * *_config->angleGamma / (dtime * dtime);

#else
        /** Position **/
        // Update model
        posA(0, 1) = dtime;
        posA(0, 2) = 0.5 * dtime * dtime;

        posA(1, 2) = dtime;

        posA(3, 4) = dtime;
        posA(3, 5) = 0.5 * dtime * dtime;

        posA(4, 5) = dtime;

        // Position
        posZ(0) = observedPos.x;
        posZ(1) = observedPos.y;
        // RobotModel::shared &robot = p.second;

        _posKalman->predict(&posU);
        _posKalman->correct(&posZ);

        _pos.x = (float)_posKalman->state()->elt(0);
        _vel.x = (float)_posKalman->state()->elt(1);
        _accel.x = (float)_posKalman->state()->elt(2);
        _pos.y = (float)_posKalman->state()->elt(3);
        _vel.y = (float)_posKalman->state()->elt(4);
        _accel.y = (float)_posKalman->state()->elt(5);

        // Using the ABG filter for independent velocity and acceleration
        // 		Geometry2d::Point predictPos = abgPos + vel * dtime + accel * 0.5f
        // * dtime * dtime;
        //         Geometry2d::Point predictVel = vel + accel * dtime;
        //
        // 		Geometry2d::Point posError = observedPos - predictPos;
        //         abgPos = predictPos + posError * posAlpha;
        //         vel = predictVel + posError * posBeta / dtime;
        //         accel += posError * posGamma / (dtime * dtime);

        /** Angle **/
        // Update model
        angA(0, 1) = dtime;
        angA(0, 2) = 0.5 * dtime * dtime;

        angA(1, 2) = dtime;

        angB(0, 1) = angB(1, 2) = dtime;
        angB(1, 1) = 1;

        // Position
        angZ(0) = observedAngle;

        _angKalman->predict(&angU);
        _angKalman->correct(&angZ);

        _angle = (float)_angKalman->state()->elt(0);
        _angleVel = (float)_angKalman->state()->elt(1);
        _angleAccel = (float)_angKalman->state()->elt(2);

#endif

        lastUpdatedTime = cur_time;
    }

    if (verbose)
        cout << "   Final RobotModel: (" << _pos.x << ", " << _pos.y << ")"
             << endl;

    // cleanup by removing old observations
    _observations.clear();
}
