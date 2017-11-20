#include "IndependentMultiRobotPathPlanner.hpp"
#include "RobotConstraints.hpp"

using namespace std;
namespace Planning {

std::map<int, std::unique_ptr<Path>> IndependentMultiRobotPathPlanner::run(
    std::map<int, PlanRequest> requests) {
    std::map<int, std::unique_ptr<Path>> paths;

    std::map<int, shared_ptr<Geometry2d::Circle>> staticRobotObstacles;
    std::vector<int> staticRequests;
    std::vector<int> dynamicRequests;
    for (auto& entry : requests) {
        int shell = entry.first;
        PlanRequest& request = entry.second;
        auto& prevPlanner = _planners[shell];

        // Make sure we have the right planner.  If it changes from last time,
        // delete the old path.
        if (!prevPlanner ||
            prevPlanner->commandType() !=
                request.motionCommand->getCommandType()) {
            _planners[shell] =
                PlannerForCommandType(request.motionCommand->getCommandType());
            request.prevPath = nullptr;
        }

        if (_planners[shell]->canHandleDynamic()) {
            dynamicRequests.push_back(shell);
        } else {
            staticRequests.push_back(shell);
        }
        staticRobotObstacles[shell] = std::make_shared<Geometry2d::Circle>(
            request.start.pos, Robot_Radius);
    }

    // Sorts descending so that higher priorities are first
    auto comparator = [&](int& lhs, int& rhs) {
        return requests.at(lhs).priority > requests.at(rhs).priority;
    };

    std::sort(std::begin(staticRequests), std::end(staticRequests), comparator);
    std::sort(std::begin(dynamicRequests), std::end(dynamicRequests),
              comparator);

    std::vector<int> inOrderRequests;
    inOrderRequests.insert(std::end(inOrderRequests),
                           std::begin(staticRequests),
                           std::end(staticRequests));
    inOrderRequests.insert(std::end(inOrderRequests),
                           std::begin(dynamicRequests),
                           std::end(dynamicRequests));

    vector<DynamicObstacle> ourRobotsObstacles;
    for (int shell : inOrderRequests) {
        PlanRequest& request = requests.at(shell);

        if (_planners[shell]->canHandleDynamic()) {
            std::copy(std::begin(ourRobotsObstacles),
                      std::end(ourRobotsObstacles),
                      std::back_inserter(request.dynamicObstacles));
        } else {
            for (auto& entry : staticRobotObstacles) {
                if (entry.first != shell) {
                    request.obstacles.add(entry.second);
                }
            }
            SingleRobotPathPlanner::allDynamicToStatic(
                request.obstacles, request.dynamicObstacles);
            request.dynamicObstacles = std::vector<DynamicObstacle>();
        }

        paths[shell] = _planners[shell]->run(request);

        // Add our generated path to our list of our Robot Obstacles
        ourRobotsObstacles.push_back(DynamicObstacle(
            request.start.pos, Robot_Radius, paths[shell].get()));
    }

    return paths;
}

}  // namespace Planning
