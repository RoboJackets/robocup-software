#pragma once
#include <rj_geometry/arc.hpp>
#include <rj_geometry/line.hpp>
#include <rj_geometry/circle.hpp>

namespace planning {

    struct ConicalDynamicObstacle {
        rj_geometry::Circle obstacle;
        rj_geometry::Arc time_horizon;
        rj_geometry::Line 
    };
}