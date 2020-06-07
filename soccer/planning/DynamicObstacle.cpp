#include "DynamicObstacle.hpp"

#include "planning/paths/Path.hpp"

using namespace std;
using namespace geometry2d;

namespace Planning {
DynamicObstacle::DynamicObstacle(const Path* path, float radius)
    : staticPoint(path->start().motion.pos),
      path(path),
      radius(radius),
      staticObstacle(std::make_shared<geometry2d::Circle>(
          path->start().motion.pos, radius)){};

}  // namespace Planning
