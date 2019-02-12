#pragma once

#include <stdint.h>
#include "RobotModel.hpp"
#include "BallModel.hpp"
#include "rbpf/Rbpf.hpp"

class Configuration;

namespace Modeling {
class RBPFBallModel : public BallModel {
public:
    RBPFBallModel(RobotModel::RobotMap* robotMap, Configuration* config);
    virtual ~RBPFBallModel();

protected:
    // new particle filter implementation
    Rbpf* raoBlackwellizedParticleFilter;

    /** implementation of the update function  - uses multiple observations */
    virtual void update(float dtime);

    /** simpler update for a single observation */
    void singleUpdate(float dtime);
};
}
