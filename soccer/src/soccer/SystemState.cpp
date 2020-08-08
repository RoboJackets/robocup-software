#include <Robot.hpp>
#include <SystemState.hpp>

SystemState::SystemState(Context* const context) {
    // FIXME - boost::array?
    paused = false;
    self.resize(kNumShells);
    opp.resize(kNumShells);
    for (int i = 0; i < kNumShells; ++i) {
        self[i] = new OurRobot(context, i);      // NOLINT
        opp[i] = new OpponentRobot(context, i);  // NOLINT
    }

    ball = &(context->world_state.ball);
}

SystemState::~SystemState() {
    for (int i = 0; i < kNumShells; ++i) {
        delete self[i];  // NOLINT
        delete opp[i];   // NOLINT
    }
}

std::vector<int> SystemState::our_valid_ids() {
    std::vector<int> valid_ids;
    for (auto& i : self) {
        if (i->visible()) {
            valid_ids.push_back(i->shell());
        }
    }
    return valid_ids;
}
