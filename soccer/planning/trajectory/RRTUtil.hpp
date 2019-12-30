#include <Geometry2d/Point.hpp>
#include <rrt/BiRRT.hpp>
#include <DebugDrawer.hpp>
#include "Configuration.hpp"
#include "SystemState.hpp"
#include "RoboCupStateSpace.hpp"
#include "planning/MotionConstraints.hpp"
#include "planning/trajectory/Trajectory.hpp"

namespace Planning {
class RRTConfig {
public:
    static void createConfiguration(Configuration* cfg);

    // if set, enables drawng of rrts to the SystemState so they can be shown in
    // the gui
    static ConfigBool* EnableRRTDebugDrawing;

    static ConfigDouble* StepSize;
    static ConfigDouble* GoalBias;
    static ConfigDouble* WaypointBias;

    static ConfigInt* MinIterations;
    static ConfigInt* MaxIterations;
};

/// Drawing
void DrawRRT(const RRT::Tree<Geometry2d::Point>& rrt, DebugDrawer* debug_drawer,
             unsigned shellID);
void DrawBiRRT(const RRT::BiRRT<Geometry2d::Point>& biRRT,
               DebugDrawer* debug_drawer, unsigned shellID);

/**
 * Generate a non-smooth path with BiRRT. This will just generate the path
 * through the tree - the user is responsible for removing points later.
 *
 * @param start The starting position.
 * @param goal The goal position.
 * @param state_space The state space to use. See @ref CreateStateSpace.
 * @param waypoints A vector of points from a previous path. The RRT will be
 *      biased towards these points. If empty, they will be unused.
 * @return A vector of points representing some clear path from the start to
 *      the end.
 */
std::vector<Geometry2d::Point> GenerateRRT(
        Geometry2d::Point start,
        Geometry2d::Point goal,
        const std::shared_ptr<RoboCupStateSpace>& state_space,
        const std::vector<Geometry2d::Point>& waypoints = {});

Trajectory RRTTrajectory(const RobotInstant& start, const RobotInstant& goal, const MotionConstraints& motionConstraints, const Geometry2d::ShapeSet& static_obstacles, const std::vector<DynamicObstacle>& dynamic_obstacles = {},const std::vector<Geometry2d::Point>& biasWaypoints = {});

}  // Planning
