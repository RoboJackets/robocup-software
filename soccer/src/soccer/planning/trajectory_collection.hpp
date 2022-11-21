#pragma once

#include "trajectory.hpp"
#include "rj_constants/constants.hpp"

namespace planning {

// Per-robot (trajectory, priority)
using Entry = std::tuple<std::shared_ptr<const Trajectory>, int>;

/**
 * A collection of per-robot trajectories.
 */
class TrajectoryCollection {
public:
    std::array<Entry, kNumShells> get();

    Entry get(int robot_id);

    void put(int robot_id, std::shared_ptr<const Trajectory> trajectory, int priority);

private:
    std::mutex lock_;
    std::array<std::mutex, kNumShells> entry_locks;
    std::array<Entry, kNumShells> robot_trajectories_ = {};
};

}
