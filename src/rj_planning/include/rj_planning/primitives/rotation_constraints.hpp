#pragma once

namespace planning {

struct RotationConstraints {
    RotationConstraints()
        : max_speed(3.0),
          max_accel(3.0) {}
    double max_speed;
    double max_accel;
};

}  // namespace planning
