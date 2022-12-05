#include "planning/trajectory_collection.hpp"

namespace planning {

std::array<Entry, kNumShells> TrajectoryCollection::get() {
    std::lock_guard lock(lock_);
    return robot_trajectories_;
}

Entry TrajectoryCollection::get(int robot_id) {
    return robot_trajectories_.at(robot_id);
}

void TrajectoryCollection::put(int robot_id, std::shared_ptr<const Trajectory> trajectory,
                               int priority) {
    // associate a (Trajectory, priority) tuple with a robot id
    std::lock_guard lock(entry_locks.at(robot_id));
    robot_trajectories_.at(robot_id) = std::make_tuple(std::move(trajectory), priority);
}

void TrajectoryCollection::lock(int robot_id) {
    auto& mutex = entry_locks.at(robot_id);
    mutex.lock();
}

void TrajectoryCollection::unlock(int robot_id) {
    auto& mutex = entry_locks.at(robot_id);
    mutex.unlock();
}

}  // namespace planning
