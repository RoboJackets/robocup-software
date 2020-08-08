#include "rj_common/FieldDimensions.hpp"

const FieldDimensions FieldDimensions::kSingleFieldDimensions(6.050f, 4.050f, 0.250f, 0.010f,
                                                                0.700f, 0.180f, 0.160f, 1.000f,
                                                                2.000f, 0.500f, 1.000f, 0.350f,
                                                                6.550f, 4.550f);

const FieldDimensions FieldDimensions::kDoubleFieldDimensions(9.000f, 6.000f, 0.700f, 0.010f,
                                                                1.000f, 0.180f, 0.160f, 1.000f,
                                                                2.000f, 0.500f, 1.000f, 0.500f,
                                                                10.400f, 7.400f);

const FieldDimensions FieldDimensions::kDefaultDimensions =
    FieldDimensions::kDoubleFieldDimensions;

FieldDimensions FieldDimensions::current_dimensions = FieldDimensions::kDefaultDimensions;
