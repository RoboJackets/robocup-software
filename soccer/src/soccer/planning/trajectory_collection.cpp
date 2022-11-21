#include "planning/trajectory_collection.hpp"

namespace planning {

std::array<Entry, kNumShells> TrajectoryCollection::get() {
    std::lock_guard lock(lock_);
    return robot_trajectories_;
}

Entry TrajectoryCollection::get(int robot_id) {
    std::lock_guard(entry_locks.at(robot_id));
    // TODO: return the whole tuple?
    return robot_trajectories_.at(robot_id);
}
// TODO: add "copy()" function here

void TrajectoryCollection::put(int robot_id, std::shared_ptr<const Trajectory> trajectory, int priority) {
    // associate a (Trajectory, priority) tuple with a robot id
    std::lock_guard lock(entry_locks.at(robot_id));
    robot_trajectories_.at(robot_id) = std::make_tuple(std::move(trajectory), priority);
}

}
