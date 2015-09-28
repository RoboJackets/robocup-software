
#pragma once

#include <stdint.h>
#include <memory>
#include <vector>

#include <Geometry2d/Point.hpp>
#include "RobotModel.hpp"

class Configuration;

namespace Modeling {
/**
 * This base class allows for different implementations without redundant memory
 * allocations
 */
class BallModel {
public:
    // Maximum time to coast a track (keep the track alive with no observations)
    // in microseconds.
    static const uint64_t MaxCoastTime = 100000;  // tenth of a second

    typedef enum { VISION, BALL_SENSOR } observation_mode;

    BallModel(RobotModel::RobotMap* robotMap, Configuration* config);
    virtual ~BallModel();

    void observation(uint64_t time, const Geometry2d::Point& pos,
                     observation_mode obs_mode);

    bool valid(uint64_t time);

    Geometry2d::Point predictPosAtTime(float dtime);

    /** This function is called by outside functions */
    void run(uint64_t time);

    /** state of filter */
    Geometry2d::Point pos, vel, accel;

    // Best observation so far
    Geometry2d::Point observedPos;

    uint64_t lastObservedTime, lastUpdatedTime;
    int missedFrames;

    Geometry2d::Point prevObservedPos;

protected:
    typedef struct {
        uint64_t time;
        Geometry2d::Point pos;
        observation_mode obs_type;
    } observation_type;

    std::vector<observation_type> _observations;

    Configuration* _configuration;

    // map of robots so filters can include this information
    RobotModel::RobotMap* _robotMap;

    /** manages updating given a set of observations */
    virtual void update(float dtime) = 0;

    /**
     * perform initializations of params from configs
     * Called every frame, but does nothing in default implementation
     */
    virtual void initParams() {}
};
}
