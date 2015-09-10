#pragma once

class Path;
namespace Planning {


class RotationPlanner {
public:
    virtual void run(Path path) = 0;
};

}  // namespace Planning
