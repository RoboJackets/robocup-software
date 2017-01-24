#include "Path.hpp"
#include <protobuf/LogFrame.pb.h>
#include "SystemState.hpp"
#include "DynamicObstacle.hpp"
#include "Geometry2d/ShapeSet.hpp"

using namespace std;
using namespace Geometry2d;
namespace Planning {

class ConstPathIterator;

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
    const RJ::Seconds duration = getDuration();
    const RJ::Seconds desiredStep = 250ms;

    // draw the path by interpolating every x seconds
    const float segmentCount = roundf(duration / desiredStep);
    const RJ::Seconds step = duration / segmentCount;

    // Draw points along the path except the last one
    for (int i = 0; i < segmentCount; ++i) {
        RJ::Seconds t = i * step;
        addPoint(evaluate(t)->motion);
    }

    // Draw the last point of the path
    addPoint(end().motion);
}

void Path::drawDebugText(SystemState* state, const QColor& color,
                         const QString& layer) const {
    if (_debugText) {
        state->drawText(_debugText.get(), end().motion.pos, color, layer);
    }
}

std::unique_ptr<ConstPathIterator> Path::iterator(RJ::Time startTime,
                                                  RJ::Seconds deltaT) const {
    return std::move(
        std::make_unique<ConstPathIterator>(this, startTime, deltaT));
}

bool Path::pathsIntersect(const std::vector<DynamicObstacle>& obstacles,
                          RJ::Time startTime, Geometry2d::Point* hitLocation,
                          RJ::Seconds* hitTime) const {
    const RJ::Seconds deltaT = RJ::Seconds(0.05);

    auto thisPathIterator = iterator(startTime, deltaT);
    vector<std::pair<unique_ptr<ConstPathIterator>, float>> pathIterators;
    for (const auto& obs : obstacles) {
        if (obs.hasPath()) {
            pathIterators.emplace_back(
                obs.getPath()->iterator(startTime, deltaT), obs.getRadius());
        } else {
            ShapeSet set;
            set.add(obs.getStaticObstacle());
            if (hit(set, startTime - this->startTime(), hitTime)) {
                return true;
            }
        }
    }

    RJ::Seconds time = startTime - this->startTime();
    for (; time < getDuration(); time += deltaT) {
        auto current = **thisPathIterator;
        for (auto& pair : pathIterators) {
            auto& it = pair.first;
            assert(it != nullptr);
            auto hitRadius = pair.second + Robot_Radius;
            auto robotInstant = (**it);
            if (current.motion.pos.distTo(robotInstant.motion.pos) <
                hitRadius) {
                if (hitTime) {
                    *hitTime = time;
                }
                if (hitLocation) {
                    *hitLocation = std::move(robotInstant.motion.pos);
                }
                return true;
            }
            it->operator++();
        }
        thisPathIterator->operator++();
    }
    return false;
}
}
