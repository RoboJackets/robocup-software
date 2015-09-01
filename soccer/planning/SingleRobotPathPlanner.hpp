#pragma once

#include <Configuration.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/Path.hpp>

namespace Planning {

/**
 * @brief Interface for Path Planners
 */
class SingleRobotPathPlanner {
public:
    /**
     * Returns an obstacle-free Path subject to the specified MotionContraints.
     */
    virtual std::unique_ptr<Path> run(
        MotionInstant startInstant, MotionCommand cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) = 0;

    static double goalChangeThreshold() { return *_goalChangeThreshold; }
    static double replanTimeout() { return *_replanTimeout; }

    static void createConfiguration(Configuration* cfg);

private:
    static ConfigDouble* _goalChangeThreshold;
    static ConfigDouble* _replanTimeout;
};

}  // namespace Planning
