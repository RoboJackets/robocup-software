#include <Geometry2d/Point.hpp>
#include <rrt/BiRRT.hpp>
#include "Configuration.hpp"
#include "SystemState.hpp"

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
};

/// Drawing
void DrawRRT(const RRT::Tree<Geometry2d::Point>& rrt, DebugDrawer* debug_drawer,
             unsigned shellID);
void DrawBiRRT(const RRT::BiRRT<Geometry2d::Point>& biRRT,
               DebugDrawer* debug_drawer, unsigned shellID);
}  // Planning
