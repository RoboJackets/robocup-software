#include "BallModel.hpp"

#include <Configuration.hpp>

#include <iostream>

using namespace std;

Modeling::BallModel::BallModel(mode_t mode, RobotModel::RobotMap* robotMap,
                               Configuration* config)
    : A(6, 6),
      B(6, 6),
      P(6, 6),
      Q(6, 6),
      R(2, 2),
      H(2, 6),
      Z(2),
      U(6),
      X0(6),
      _mode(mode),
      _robotMap(robotMap) {
    lastObservedTime = 0;
    lastUpdatedTime = 0;
    raoBlackwellizedParticleFilter = 0;

    if (_mode == MODELTESTS) {
        cout << "Running ball model tests..." << endl;
        initKalman();
        initRBPF(config);
        kalmanTestPosError = 0;
        kalmanTestVelError = 0;
        rbpfTestPosError = 0;
        rbpfTestVelError = 0;
    } else if (_mode == KALMAN) {
        initKalman();
    } else if (_mode == RBPF) {
        initRBPF(config);
    } else if (_mode == ABG) {
        initABG();
    } else {
        cout << "ERROR: Invalid initialization type, defaulting to RPBF!"
             << endl;
        _mode = RBPF;
        initRBPF(config);
    }
}

Modeling::BallModel::~BallModel() {
    if (raoBlackwellizedParticleFilter) {
        delete raoBlackwellizedParticleFilter;
    }
}

void Modeling::BallModel::initABG() {
    alpha = 1;
    beta = .4;
    gamma = .1;
}

void Modeling::BallModel::initKalman() {
    A.zero();
    B.zero();
    Q.zero();
    R.zero();
    H.zero();
    P.zero();
    X0.zero();

    // Process covariance between position and velocity (E[x,x_dot])
    // 	Q(1,0) = Q(4,3) = 0.01;

    // Process covariance between velocity and acceleration (E[x)dot,x_ddot])
    // 	Q(2,1) = Q(5,4) = 0.001;

    // Process covariance between position and acceleration (E[x,x_ddot])
    // 	Q(2,0) = Q(5,3) = 0.0001;

    // Process covariance (E[x,x], E[x_dot,x_dot], etc)
    Q(0, 0) = Q(3, 3) = 10;

    Q(1, 1) = Q(4, 4) = 10;

    Q(2, 2) = Q(5, 5) = 10;

    // Measurement Covariance for position in the x and the y
    R(0, 0) = R(1, 1) = 0.84;

    // Measurement Model. We can only measure position
    H(0, 0) = H(1, 3) = 1;

    // State Model
    A(0, 0) = A(1, 1) = A(2, 2) = A(3, 3) = A(4, 4) = A(5, 5) = 1;

    posKalman = new DifferenceKalmanFilter(&A, &B, &X0, &P, &Q, &R, &H);
}

void Modeling::BallModel::initRBPF(Configuration* config) {
    // Construct initial state X (n x 1)
    Vector X(6);
    X.clear();
    // Construct initial state covariance P (n x n)
    Matrix P(6, 6);
    P.clear();
    P(0, 0) = P(1, 1) = P(2, 2) = P(3, 3) = P(4, 4) = P(5, 5) = 0.01;
    // Create Rbpf
    int numParticles = 20;  // Number of particles in filter
    raoBlackwellizedParticleFilter = new Rbpf(X, P, numParticles);
    // create model graph
    raoBlackwellizedParticleFilter->addModel(
        new RbpfModelRolling(_robotMap, config));
    raoBlackwellizedParticleFilter->addModel(
        new RbpfModelKicked(_robotMap, config));
    raoBlackwellizedParticleFilter->setTransProb(0, 0, 0.9);
    raoBlackwellizedParticleFilter->setTransProb(0, 1, 0.1);
    raoBlackwellizedParticleFilter->setTransProb(1, 0, 0.9);
    raoBlackwellizedParticleFilter->setTransProb(1, 1, 0.1);
}

Geometry2d::Point Modeling::BallModel::predictPosAtTime(float dtime) {
    // return pos + vel * dtime + accel * 0.5f * dtime * dtime;
    // because the approximation of accel is poor, just predict based on
    // velocity and position
    return pos + vel * dtime;
}

void Modeling::BallModel::observation(uint64_t time,
                                      const Geometry2d::Point& pos,
                                      observation_mode obs_type) {
    if (time < lastUpdatedTime) {
        // due to an issue with early measurements coming in late, this will
        // discard any observations that happened before the previous update
        return;
    }

    observation_type obs = {time, pos, obs_type};
    _observations.push_back(obs);

    // set lastObservedTime to the last observation's time
    if (time >= lastObservedTime) lastObservedTime = time;
}

void Modeling::BallModel::kalmanUpdate(float dtime) {
    // Update model
    A(0, 1) = dtime;
    A(0, 2) = 0.5 * dtime * dtime;

    A(1, 2) = dtime;

    A(3, 4) = dtime;
    A(3, 5) = 0.5 * dtime * dtime;

    A(4, 5) = dtime;

    // Position
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
}

void Modeling::BallModel::rbpfUpdate(float dtime) {
    raoBlackwellizedParticleFilter->update(observedPos.x, observedPos.y, dtime);
    RbpfState* bestState = raoBlackwellizedParticleFilter->getBestFilterState();
    Point posOld = pos;
    pos.x = bestState->X(0);
    pos.y = bestState->X(1);
    vel.x = bestState->X(2);
    vel.y = bestState->X(3);
    accel.x = bestState->X(4);
    accel.y = bestState->X(5);
}

void Modeling::BallModel::rbpfUpdateMultipleObs(
    std::vector<observation_type>& obs) {
    if (obs.size() >= 1) {  // currently hacked to just handle a single update
        // pick the closest observation to the current estimate
        float bestDist = 99999;
        for (const observation_type& observation : _observations) {
            if (observation.pos.distTo(pos) < bestDist) {
                bestDist = observation.pos.distTo(pos);
                observedPos = observation.pos;
            }
        }
        float dtime = (float)(obs.at(0).time - lastUpdatedTime) / 1e6;
        rbpfUpdate(dtime);
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

void Modeling::BallModel::abgUpdate(float dtime) {
    Geometry2d::Point predictPos =
        abgPos + vel * dtime + accel * 0.5f * dtime * dtime;
    Geometry2d::Point predictVel = vel + accel * dtime;

    Geometry2d::Point posError = observedPos - predictPos;
    abgPos = predictPos + posError * alpha;
    vel = predictVel + posError * beta / dtime;
    accel += posError * gamma / (dtime * dtime);
}

bool Modeling::BallModel::valid(uint64_t time) {
    // cout << "ball model is " << (!_observations.empty() || ((time -
    // lastUpdatedTime) < MaxCoastTime) ? "" : "not") << " valid" << std::endl;
    return !_observations.empty() && ((time - lastUpdatedTime) < MaxCoastTime);
}

void Modeling::BallModel::run(uint64_t time) {
    const bool verbose = false;

    if (_observations.empty()) {
        // coast the ball using simple integrator
        float dtime = (float)(time - lastUpdatedTime) / 1e6;
        pos = predictPosAtTime(dtime);
        vel += accel * 0.5f * dtime * dtime;
        return;
    }

    // loop through the observations and determine if we got vision observations
    bool gotCameraObservation = false;
    for (const observation_type& obs : _observations) {
        if (obs.obs_type == BallModel::VISION) {
            gotCameraObservation = true;
            break;
        }
    }

    // If there are camera observations, remove robot sensor observations.
    vector<observation_type> goodObs;
    if (gotCameraObservation) {
        for (const observation_type& obs : _observations) {
            if (obs.obs_type == BallModel::VISION) {
                goodObs.push_back(obs);
            }
        }
    } else {
        goodObs = _observations;
    }
    _observations = goodObs;
    if (verbose)
        cout << " ballModel has " << _observations.size()
             << " observations, updating..." << endl;

    // TODO: handle non-RBPF filters properly
    if (_mode != RBPF) {
        cout << "update() not implemented in ballmodel.cpp" << endl;
    } else if (_mode == RBPF) {
        rbpfUpdateMultipleObs(_observations);
    }

    // remove previous observations after processing them
    _observations.clear();

    // remember last update
    lastUpdatedTime = time;

    /*
    float dtime = (float)(bestObservedTime - lastObservedTime) / 1e6;

    if (missedFrames > 5 || bestError < (0.2 * 0.2))
    {
        lastObservedTime = bestObservedTime;

        // assuming we moved, then update the filter
        if (dtime)
        {
            if (_mode == MODELTESTS){
                Geometry2d::Point posKalmanDiff, posRbpfDiff;
                Geometry2d::Point velKalmanDiff, velRbpfDiff;
                Geometry2d::Point observedVel = (observedPos -
    prevObservedPos)*(1/dtime);
                float errorPosKalman, errorPosRbpf;
                float errorVelKalman, errorVelRbpf;

                kalmanUpdate(dtime);
                posKalmanDiff = observedPos - pos;
                velKalmanDiff = observedVel - vel;
                rbpfUpdate(dtime);
                posRbpfDiff = observedPos - pos;
                velRbpfDiff = observedVel - vel;

                errorPosKalman = posKalmanDiff.mag();
                errorPosRbpf = posRbpfDiff.mag();
                errorVelKalman = velKalmanDiff.mag();
                errorVelRbpf = velRbpfDiff.mag();

                kalmanTestPosError += errorPosKalman;
                rbpfTestPosError += errorPosRbpf;
                kalmanTestVelError += errorVelKalman;
                rbpfTestVelError += errorVelRbpf;
                //cout << "Total error (Kal, Rbpf): (" << kalmanTesPostError <<
    "," << rbpfTestPosError << "), this obs error (Kal, Rbpf): (" << errorKalman
    << "," << errorRbpf << ")" << endl;
                cout << "Total pos error (Kal, Rbpf): (" << kalmanTestPosError
    << "," << rbpfTestPosError << "), total vel error (Kal, Rbpf): (" <<
    kalmanTestVelError << "," << rbpfTestVelError << ")" << endl;
            } else if (_mode == KALMAN) {
                kalmanUpdate(dtime);
            } else if (_mode == ABG) {
                abgUpdate(dtime);
            } else if (_mode == RBPF) {
                rbpfUpdate(dtime);
            }
            prevObservedPos = observedPos;
        }
    } else {
        // Ball moved too far to possibly be a valid track, so just extrapolate
    from the last known state
        if (dtime)
        {
            if(_mode == MODELTESTS){
                cout << "Ball Model Tests: missed too many frames,
    extrapolating." << endl;
            }
            pos += vel * dtime + accel * 0.5f * dtime * dtime;
        }

        ++missedFrames;
    }
    bestError = -1;
    */
}
