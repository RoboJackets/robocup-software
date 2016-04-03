#pragma once

#include <rrt/2dplane/PlaneStateSpace.hpp>
#include <Geometry2d/Point.hpp>

namespace Planning {

class RoboCupStateSpace : public RRT::PlaneStateSpace<Geometry2d::Point> {
public:
    // TODO: take Field_Dimensions as a parameter
    RoboCupStateSpace(float width, float height)
        : PlaneStateSpace(width, height) {}

    Geometry2d::Point randomState() const;

    bool stateValid(const Geometry2d::Point& state) const;

    bool transitionValid(const Geometry2d::Point& from,
                         const Geometry2d::Point& to) const;
};

}  // Planning
