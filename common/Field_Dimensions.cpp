#include "Field_Dimensions.hpp"

const Field_Dimensions Field_Dimensions::Single_Field_Dimensions(
    6.050f, 4.050f, 0.250f, 0.010f, 0.700f, 0.180f, 0.160f, 0.750f, 0.010f,
    0.800f, 0.500f, 1.000f, 0.350f, 6.550f, 4.550f);

const Field_Dimensions Field_Dimensions::Double_Field_Dimensions(
    9.000f, 6.000f, 0.700f, 0.010f, 1.000f, 0.180f, 0.160f, 1.000f, 0.010f,
    1.000f, 0.500f, 1.000f, 0.500f, 10.400f, 7.400f);

const Field_Dimensions Field_Dimensions::Default_Dimensions =
    Field_Dimensions::Double_Field_Dimensions;

Field_Dimensions Field_Dimensions::Current_Dimensions =
    Field_Dimensions::Default_Dimensions;
