#pragma once

namespace action_behavior_server {

enum Subsystem {
    kDrivetrain = 1,
    kDribbler = 2,
    kKicker = 4,
};

struct Resource {
    Subsystem system;
};

}  // namespace action_behavior_server