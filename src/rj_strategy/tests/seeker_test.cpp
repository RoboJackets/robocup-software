#include "rj_strategy/agent/position/seeker.hpp"

#include <gtest/gtest.h>

#include <rj_common/field_dimensions.hpp>

using namespace strategy;

// Verify that the field dimension values used by the seeker constraint are
// consistent: center_field_loc is midway between goals, and strictly above
// our goal line.
TEST(SeekerTest, correct_point_constraint_values) {
    FieldDimensions dims = FieldDimensions::kDefaultDimensions;

    double midfield_y = dims.center_field_loc().y();
    double expected_center = (dims.our_goal_loc().y() + dims.their_goal_loc().y()) / 2.0;

    // The minimum Y used in correct_point must equal the field center.
    EXPECT_DOUBLE_EQ(midfield_y, expected_center);

    // Center field Y must be strictly greater than our goal Y (i.e. the
    // constraint actually excludes our half of the field).
    EXPECT_GT(midfield_y, dims.our_goal_loc().y());

    // Center field Y must be strictly less than their goal Y.
    EXPECT_LT(midfield_y, dims.their_goal_loc().y());
}
