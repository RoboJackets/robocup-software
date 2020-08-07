#include "rj_common/Field_Dimensions.hpp"

const Field_Dimensions Field_Dimensions::kSingleFieldDimensions(6.050f, 4.050f, 0.250f, 0.010f,
                                                                0.700f, 0.180f, 0.160f, 1.000f,
                                                                2.000f, 0.500f, 1.000f, 0.350f,
                                                                6.550f, 4.550f);

const Field_Dimensions Field_Dimensions::kDoubleFieldDimensions(9.000f, 6.000f, 0.700f, 0.010f,
                                                                1.000f, 0.180f, 0.160f, 1.000f,
                                                                2.000f, 0.500f, 1.000f, 0.500f,
                                                                10.400f, 7.400f);

const Field_Dimensions Field_Dimensions::kDefaultDimensions =
    Field_Dimensions::kDoubleFieldDimensions;

Field_Dimensions Field_Dimensions::current_dimensions = Field_Dimensions::kDefaultDimensions;
