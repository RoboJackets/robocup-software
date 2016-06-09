#pragma once

#include <Configuration.hpp>
#include <planning/MotionConstraints.hpp>
#include <planning/MotionCommand.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/Path.hpp>
#include "planning/RotationCommand.hpp"
#include "planning/DynamicObstacle.hpp"
#include "RobotConstraints.hpp"

namespace Planning {

struct SinglePlanRequest {
    SinglePlanRequest(const MotionInstant& startInstant,
                      const MotionCommand& cmd,
                      const RobotConstraints& robotConstraints,
                      Geometry2d::ShapeSet& obstacles,
                      const std::vector<DynamicObstacle>& dynamicObstacles,
                      const SystemState& systemState,
                      std::unique_ptr<Path> prevPath)
        : startInstant(startInstant),
          cmd(cmd),
          robotConstraints(robotConstraints),
          obstacles(obstacles),
          dynamicObstacles(dynamicObstacles),
          systemState(systemState),
          prevPath(std::move(prevPath)){};

    const MotionInstant& startInstant;
    const MotionCommand& cmd;
    const RobotConstraints& robotConstraints;
    Geometry2d::ShapeSet& obstacles;
    const std::vector<DynamicObstacle>& dynamicObstacles;
    const SystemState& systemState;
    std::unique_ptr<Path> prevPath = nullptr;
};

/**
 * @brief Interface for Path Planners
 */
class SingleRobotPathPlanner {
public:
    /**
     * Returns an obstacle-free Path subject to the specified MotionContraints.
     */
    virtual std::unique_ptr<Path> run(SinglePlanRequest& planRequest) = 0;

    /// The MotionCommand type that this planner handles
    virtual MotionCommand::CommandType commandType() const = 0;

    static double goalChangeThreshold() { return *_goalChangeThreshold; }
    static double replanTimeout() { return *_replanTimeout; }

    static void createConfiguration(Configuration* cfg);

    // Adds all static obstacle portions of the dynamic obstacle to static
    // obstacles
    static void allDynamicToStatic(
        Geometry2d::ShapeSet& obstacles,
        const std::vector<DynamicObstacle>& dynamicObstacles);

    static void splitDynamic(
        Geometry2d::ShapeSet& obstacles, std::vector<const Path*>& dynamicOut,
        const std::vector<DynamicObstacle>& dynamicObstacles);
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
    static bool shouldReplan(const SinglePlanRequest& planRequest);

    virtual bool canHandleDynamic() { return handlesDynamic; }

protected:
    SingleRobotPathPlanner(bool handlesDynamic)
        : handlesDynamic(handlesDynamic) {}

private:
    static ConfigDouble* _goalChangeThreshold;
    static ConfigDouble* _replanTimeout;

    const bool handlesDynamic;
};

/// Gets the subclass of SingleRobotPathPlanner responsible for handling the
/// given command type.  Any new SingleRobotPathPlanner classes should be
/// registered by placing them in this function's implementation in the .cpp
/// file.
std::unique_ptr<Planning::SingleRobotPathPlanner> PlannerForCommandType(
    Planning::MotionCommand::CommandType type);

boost::optional<std::function<AngleInstant(MotionInstant)>>
angleFunctionForCommandType(const Planning::RotationCommand& command);
}  // namespace Planning
