
#include <iostream>

#include "EKFBallModel.hpp"

/* Lin Alg Includes */
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

using namespace std;
using namespace Geometry2d;

Modeling::EKFBallModel::EKFBallModel(RobotModel::RobotMap* robotMap,
                                     Configuration* config)
    : BallModel(robotMap, config),
      _processNoiseSqrdPos(config, "EKFModelBall/Process Noise Position", 0.2),
      _processNoiseSqrdVel(config, "EKFModelBall/Process Noise Velocity", 1.0),
      _measurementNoiseSqrd(config, "EKFModelBall/Measurement Noise Position",
                            0.01) {
    typedef boost::numeric::ublas::vector<double> Vector;
    typedef boost::numeric::ublas::matrix<double> Matrix;

    _ekf = new ExtendedKalmanFilter(Q, R);
}

Modeling::EKFBallModel::~EKFBallModel() {
    if (_ekf) {
        delete _ekf;
    }
}

void Modeling::EKFBallModel::singleUpdate(float dtime) {
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

void Modeling::EKFBallModel::update(float dtime) {
    if (_observations.size() >=
        1) {  // currently hacked to just handle a single update
        // pick the closest observation to the current estimate
        float bestDist = 99999;
        for (const observation_type& observation : _observations) {
            if (observation.pos.distTo(pos) < bestDist) {
                bestDist = observation.pos.distTo(pos);
                observedPos = observation.pos;
            }
        }
        float dtime = (float)(_observations.at(0).time - lastUpdatedTime) / 1e6;
        singleUpdate(dtime);
    }
}

void Modeling::EKFBallModel::initParams();
