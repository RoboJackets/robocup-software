#pragma once

#include <Geometry2d/Point.hpp>
#include <rrt/2dplane/PlaneStateSpace.hpp>

namespace Planning {

class RoboCupStateSpace : public RRT::PlaneStateSpace<Geometry2d::Point> {
public:
    // TODO: take Field_Dimensions as a parameter
    RoboCupStateSpace(const Field_Dimensions& dims) : PlaneStateSpace(1, 1) {}

    Geometry2d::Point randomState() const {
        return Geometry2d::Point();  // FIX
    }

    bool stateValid(const Geometry2d::Point& state) const {
        return true;  // FIX
    }

    Geometry2d::Point intermediateState(const Geometry2d::Point& source,
                                        const Geometry2d::Point& target,
                                        float minStepSize,
                                        float maxStepSize) const {
        return Geometry2d::Point();  // FIX
    }

    bool transitionValid(const Geometry2d::Point& from,
                         const Geometry2d::Point& to) const {
        return true;  // FIX
    }
};

}  // Planning
