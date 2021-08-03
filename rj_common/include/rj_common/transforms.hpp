#include "field_dimensions.hpp"

namespace rj_common {

/**
 * @brief Returns the team angle
 * @return The team angle.
 */
[[nodiscard]] double team_angle(bool defend_plus_x);

/**
 * Build the world_to_team matrix. Multiplying a point in vision coordinates (origin at the center
 * of the field) by this matrix yields a point in team coordinates (origin at our goal).
 * @param defend_plus_x Whether our (defended) goal has positive x-coordinate in the vision frame
 */
[[nodiscard]] rj_geometry::TransformMatrix world_to_team(const FieldDimensions& dimensions,
                                                         bool defend_plus_x);

}  // namespace rj_common