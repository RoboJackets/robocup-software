#include "RRTUtil.hpp"

using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(RRTConfig)

ConfigBool* RRTConfig::EnableRRTDebugDrawing;

void RRTConfig::createConfiguration(Configuration* cfg) {
    EnableRRTDebugDrawing =
        new ConfigBool(cfg, "PathPlanner/RRT/EnableDebugDrawing", false);
}

ConfigBool EnableExpensiveRRTDebugDrawing();

void DrawRRT(const RRT::Tree<Point>& rrt, SystemState* state, unsigned shellID,
             QColor color) {
    for (auto* node : rrt.allNodes()) {
        if (node->parent()) {
            state->drawLine(Segment(node->state(), node->parent()->state()),
                            color, QString("RobotRRT%1").arg(shellID));
        }
    }
}

void DrawBiRRT(const RRT::BiRRT<Point>& biRRT, SystemState* state,
               unsigned shellID) {
    DrawRRT(biRRT.startTree(), state, shellID, QColor("blue"));
    DrawRRT(biRRT.goalTree(), state, shellID, QColor("green"));
}
}
