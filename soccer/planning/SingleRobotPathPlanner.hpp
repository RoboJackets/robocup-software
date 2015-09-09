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

    /// The MotionCommand type that this planner handles
    virtual MotionCommand::CommandType commandType() const = 0;

    static double goalChangeThreshold() { return *_goalChangeThreshold; }
    static double replanTimeout() { return *_replanTimeout; }

    static void createConfiguration(Configuration* cfg);

    /// Checks if the previous path is no longer valid and needs to be
    /// re-planned.  This method does the following checks:
    /// * Is path non-null?
    /// * Does it have a valid destination()?
    /// * Is the robot too far away from where the path says it should be?  (see
    ///   the replan threshold)
    /// * Does the path enter new obstacles?
    ///
    /// Subclasses will generally use this method in addition to their own
    /// planner-specific checks to determine if a replan is necessary.
    static bool shouldReplan(MotionInstant currentInstant,
                             const MotionConstraints& motionConstraints,
                             const Geometry2d::ShapeSet* obstacles,
                             const Path* prevPath);

private:
    static ConfigDouble* _goalChangeThreshold;
    static ConfigDouble* _replanTimeout;
};

/// Gets the subclass of SingleRobotPathPlanner responsible for handling the
/// given command type.  Any new SingleRobotPathPlanner classes should be
/// registered by placing them in this function's implementation in the .cpp
/// file.
std::unique_ptr<Planning::SingleRobotPathPlanner> PlannerForCommandType(
    Planning::MotionCommand::CommandType type);

}  // namespace Planning
