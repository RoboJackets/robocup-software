#include "IndependentMultiRobotPathPlanner.hpp"

namespace Planning {

std::map<int, std::unique_ptr<Path>>
IndependentMultiRobotPathPlanner::run(std::map<int, PlanRequest> requests) {
  std::map<int, std::unique_ptr<Path>> paths;

  for (auto &entry : requests) {
    int shell = entry.first;
    PlanRequest &request = entry.second;
    auto &prevPlanner = _planners[shell];

    // Make sure we have the right planner.  If it changes from last time,
    // delete the old path.
    if (!prevPlanner ||
        prevPlanner->commandType() != request.motionCommand->getCommandType()) {
      _planners[shell] =
          PlannerForCommandType(request.motionCommand->getCommandType());
      request.prevPath = nullptr;
    }

    paths[shell] = _planners[shell]->run(
        request.start, request.motionCommand, request.constraints,
        request.obstacles.get(), std::move(request.prevPath));
  }

  return paths;
}

} // namespace Planning
