#include "../include/rj_common/transforms.hpp"

namespace rj_common {

[[nodiscard]] double team_angle(bool defend_plus_x) { return defend_plus_x ? -M_PI_2 : M_PI_2; }

[[nodiscard]] rj_geometry::TransformMatrix world_to_team(const FieldDimensions& dimensions,
                                                         bool defend_plus_x) {
    rj_geometry::TransformMatrix world_to_team =
        rj_geometry::TransformMatrix::translate(0, dimensions.length() / 2.0f);
    world_to_team *=
        rj_geometry::TransformMatrix::rotate(static_cast<float>(team_angle(defend_plus_x)));
    return world_to_team;
}

}  // namespace rj_common
