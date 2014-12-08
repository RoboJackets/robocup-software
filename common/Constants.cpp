#include "Constants.hpp"

/** Single-size field constants */
const Field_Dimensions_t Single_Field_Dimensions(
    6.050f,
    4.050f,
    0.250f,
    0.010f,
    0.700f,
    0.180f,
    0.160f,
    0.750f,
    0.010f,
    0.800f,
    0.500f,
    1.000f,
    0.350f,
    6.550f,
    4.550f
);

/** Single-size field constants */
const Field_Dimensions_t Double_Field_Dimensions(
    8.090f,
    6.050f,
    0.250f,
    0.010f,
    1.000f,
    0.180f,
    0.160f,
    1.000f,
    0.010f,
    1.000f,
    0.500f,
    1.000f,
    0.500f,
    8.590f,
    6.550f
);

// Defualt value of Field_Dimensions, defined in Constants.hpp
Field_Dimensions_t Field_Dimensions = Single_Field_Dimensions;