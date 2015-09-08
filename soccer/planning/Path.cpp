#include "Path.hpp"
#include <protobuf/LogFrame.pb.h>

namespace Planning {

void Path::draw(SystemState* const state, const QColor& color,
                const QString& layer) const {
    if (!destination()) return;

    Packet::DebugRobotPath* dbg = state->logFrame->add_debug_robot_paths();
    dbg->set_layer(state->findDebugLayer(layer));

    const float duration = getDuration();
    const float desiredStep = 0.25;
    const float segmentCount = ceilf(duration / desiredStep);
    const float step = duration / segmentCount;

    for (int i = 0; i < segmentCount; ++i) {
        float t = i * step;
        MotionInstant instant = *evaluate(t);
        Packet::DebugRobotPath::DebugRobotPathPoint* pt = dbg->add_points();
        *pt->mutable_pos() = instant.pos;
        *pt->mutable_vel() = instant.vel;
    }

    MotionInstant instant = *destination();
    Packet::DebugRobotPath::DebugRobotPathPoint* pt = dbg->add_points();
    *pt->mutable_pos() = instant.pos;
    *pt->mutable_vel() = instant.vel;
}

}  // namespace Planning
