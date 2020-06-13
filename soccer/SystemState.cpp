#include <protobuf/LogFrame.pb.h>

#include <Geometry2d/Line.hpp>
#include <Geometry2d/Polygon.hpp>
#include <LogUtils.hpp>
#include <Robot.hpp>
#include <RobotConfig.hpp>
#include <SystemState.hpp>
#include <optional>

#include "DebugDrawer.hpp"
#include "planning/DynamicObstacle.hpp"

using namespace Packet;
using namespace std;
using namespace Planning;
using namespace Geometry2d;
using Planning::LinearMotionInstant;

SystemState::SystemState(Context* const context) {
    // FIXME - boost::array?
    paused = false;
    self.resize(Num_Shells);
    opp.resize(Num_Shells);
    for (int i = 0; i < Num_Shells; ++i) {
        self[i] = new OurRobot(context, i);      // NOLINT
        opp[i] = new OpponentRobot(context, i);  // NOLINT
    }

    ball = &(context->world_state.ball);
}

SystemState::~SystemState() {
    for (int i = 0; i < Num_Shells; ++i) {
        delete self[i];  // NOLINT
        delete opp[i];   // NOLINT
    }
}

std::vector<int> SystemState::ourValidIds() {
    std::vector<int> validIds;
    for (auto& i : self) {
        if (i->visible()) {
            validIds.push_back(i->shell());
        }
    }
    return validIds;
}
