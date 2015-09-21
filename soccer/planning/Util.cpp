#include "Util.hpp"
#include "Field_Dimensions.hpp"
#include <stdlib.h>

namespace Planning {

Geometry2d::Point RandomFieldLocation() {
    const auto& dims = Field_Dimensions::Current_Dimensions;
    float x = dims.FloorWidth() * (drand48() - 0.5f);
    float y = dims.FloorLength() * drand48() - dims.Border();

    return Geometry2d::Point(x, y);
}

}  // namespace Planning
