#include "Path.hpp"
#include <protobuf/LogFrame.pb.h>
namespace Planning {
// This method is a default implementation of draw() that works by evaluating
// the path at fixed time intervals form t = 0 to t = duration.
void Path::draw(SystemState* const state, const QColor& color,
                const QString& layer) const {
    Packet::DebugRobotPath* dbg = state->logFrame->add_debug_robot_paths();
    dbg->set_layer(state->findDebugLayer(layer));

    auto addPoint = [dbg](MotionInstant instant) {
        Packet::DebugRobotPath::DebugRobotPathPoint* pt = dbg->add_points();
        *pt->mutable_pos() = instant.pos;
        *pt->mutable_vel() = instant.vel;
    };

    // Get the closest step size to a desired value that is divisible into the
    // duration
    const float duration = getDuration();
    const float desiredStep =
        0.25;  // draw the path by interpolating every x seconds
    const float segmentCount = roundf(duration / desiredStep);
    const float step = duration / segmentCount;

    // Draw points along the path except the last one
    for (int i = 0; i < segmentCount; ++i) {
        float t = i * step;
        addPoint(evaluate(t)->motion);
    }

    // Draw the last point of the path
    addPoint(end().motion);
}

}  // namespace Planning
